// test_scan_transform.cpp — the mirror + mounting-yaw math round trip.
//
// The sim generates scans with exactly the wire convention (bin i clockwise
// from the zero mark); transforming bins back to world points must land the
// returns ON the known walls, for both mirror settings. This is the check
// that catches sign errors in lidarMirror / lidarMountYawDeg — the class of
// bug tools/lidar_mirror_test.py hunts for on real hardware.

#include <gtest/gtest.h>

#include "sim/sim_robot.h"
#include "vacuum/scan_transform.h"

namespace vacuum {
namespace {

SimNoise quiet() {
  SimNoise n;
  n.enabled = false;  // ideal physics: no slip, no noise, no motor lag
  return n;
}

// Distance from a point to the nearest inner wall face of the 4 m room
// (walls 0.10 m thick -> faces at 0.1 and 3.9).
double distToWallFace(const Vec2& p) {
  const double d = std::min(std::min(std::abs(p.x - 0.1), std::abs(p.x - 3.9)),
                            std::min(std::abs(p.y - 0.1), std::abs(p.y - 3.9)));
  return d;
}

void checkScanLandsOnWalls(bool mirror, double mountYawDeg) {
  RobotConfig cfg;
  cfg.lidarMirror = mirror;
  cfg.lidarMountYawDeg = mountYawDeg;

  SimWorld world = SimWorld::emptyRoom(4.0, 0.05, 0.10);
  SimRobot robot(cfg, std::move(world), quiet(), 1);
  const Pose2D pose(2.0, 2.0, 0.3);  // off-centre, rotated
  robot.setPose(pose);

  // Settle one full scan.
  for (int i = 0; i < 40; ++i) {
    robot.setWheelRpm(0.0, 0.0);
    robot.step(0.005);
  }
  ASSERT_GT(robot.sensors().scanId, 0u);  // a revolution completed
  ASSERT_EQ(robot.sensors().ranges.size(), 360u);

  int checked = 0;
  for (int i = 0; i < 360; ++i) {
    const float range = robot.sensors().ranges[i];
    if (range <= 0.0f) continue;
    const Vec2 p = binToWorldPoint(cfg, pose, i, 360, range);
    EXPECT_LT(distToWallFace(p), 0.02)
        << "bin " << i << " mirror=" << mirror << " point (" << p.x << ", "
        << p.y << ")";
    ++checked;
  }
  EXPECT_GT(checked, 340);
}

TEST(ScanTransform, MirrorTrueRoundTripLandsOnWalls) {
  checkScanLandsOnWalls(true, 180.0);   // repo defaults (flip_lr + angle0=180)
}

TEST(ScanTransform, MirrorFalseRoundTripLandsOnWalls) {
  checkScanLandsOnWalls(false, -90.0);  // mirrored mounting + odd yaw
}

TEST(ScanTransform, BinAnglesMatchWireConvention) {
  RobotConfig cfg;  // mirror=true, mount yaw 180
  // Bin 0 points along the zero mark: base +x rotated by 180° -> -x.
  EXPECT_NEAR(binToWorldAngle(cfg, Pose2D(0, 0, 0), 0, 360), kPi, 1e-12);
  // Bin 90 is 90° clockwise from the zero mark: 180 - 90 = 90° -> +y... in
  // world terms 180° - 90° (clockwise) = 90°. Yes, +y.
  EXPECT_NEAR(binToWorldAngle(cfg, Pose2D(0, 0, 0), 90, 360), kPi / 2, 1e-12);
  // Unmirrored: bin 90 runs the other way -> 180 + 90 = 270° -> -y.
  cfg.lidarMirror = false;
  EXPECT_NEAR(binToWorldAngle(cfg, Pose2D(0, 0, 0), 90, 360), -kPi / 2, 1e-12);
}

TEST(ScanTransform, LaserOriginUsesOffset) {
  RobotConfig cfg;
  cfg.lidarOffsetXM = 0.05;
  cfg.lidarOffsetYM = -0.02;
  const Pose2D base(1.0, 1.0, kPi / 2.0);  // facing +y: offset rotates to +y/+x
  const Vec2 o = laserOrigin(cfg, base);
  EXPECT_NEAR(o.x, 1.0 + 0.02, 1e-12);
  EXPECT_NEAR(o.y, 1.0 + 0.05, 1e-12);
}

}  // namespace
}  // namespace vacuum
