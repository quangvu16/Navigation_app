// test_config.cpp — RobotConfig JSON round-trip and repo-verified defaults.

#include <gtest/gtest.h>

#include "vacuum/robot_config.h"

namespace vacuum {
namespace {

TEST(RobotConfig, DefaultsMatchRepo) {
  const RobotConfig c;
  // docs/ASSUMPTIONS.md §2 — hoverboard_base.yaml / firmware config.h.
  EXPECT_DOUBLE_EQ(c.wheelRadiusM, 0.09615);
  EXPECT_DOUBLE_EQ(c.trackWidthM, 0.18);
  EXPECT_DOUBLE_EQ(c.ticksPerRev, 90.0);
  // 2*pi*0.09615/90 = 6.713 mm per tick (prompt: ~6.7 mm).
  EXPECT_NEAR(c.metersPerTick(), 2.0 * 3.14159265358979323846 * 0.09615 / 90.0,
              1e-12);
  EXPECT_NEAR(c.metersPerTick(), 0.0067, 0.0001);
  EXPECT_EQ(c.boardScanPort, 9900);
  EXPECT_EQ(c.boardCmdPort, 9901);
  EXPECT_EQ(c.boardTelemPort, 9902);
  EXPECT_TRUE(c.lidarMirror);
  EXPECT_DOUBLE_EQ(c.lidarMountYawDeg, 180.0);
}

TEST(RobotConfig, JsonRoundTrip) {
  RobotConfig c;
  c.wheelRadiusM = 0.09;
  c.trackWidthM = 0.22;
  c.lidarMirror = false;
  c.lidarMountYawDeg = -33.5;
  c.maxLinearMps = 0.3;
  c.robotName = "Bot \"A\"";

  const std::string json = c.toJson().dump();
  const RobotConfig back = RobotConfig::fromJson(FlatJson::parse(json));

  EXPECT_DOUBLE_EQ(back.wheelRadiusM, 0.09);
  EXPECT_DOUBLE_EQ(back.trackWidthM, 0.22);
  EXPECT_FALSE(back.lidarMirror);
  EXPECT_DOUBLE_EQ(back.lidarMountYawDeg, -33.5);
  EXPECT_DOUBLE_EQ(back.maxLinearMps, 0.3);
  EXPECT_EQ(back.robotName, "Bot \"A\"");
  // Untouched keys keep defaults.
  EXPECT_DOUBLE_EQ(back.ticksPerRev, 90.0);
}

TEST(RobotConfig, JsonMissingKeysFallBackToDefaults) {
  const RobotConfig back =
      RobotConfig::fromJson(FlatJson::parse("{\"wheelRadiusM\":0.1}"));
  EXPECT_DOUBLE_EQ(back.wheelRadiusM, 0.1);
  EXPECT_DOUBLE_EQ(back.trackWidthM, 0.18);
}

}  // namespace
}  // namespace vacuum
