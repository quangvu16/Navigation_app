// test_odometry.cpp - differential-drive odometry from cumulative ticks.

#include <gtest/gtest.h>

#include "vacuum/odometry.h"

namespace vacuum {
namespace {

TEST(Odometry, StraightLineIsExact) {
  RobotConfig cfg;
  Odometry odom(cfg);
  odom.reset(Pose2D(1.0, 2.0, 0.0));
  odom.update(0, 0);   // first update after reset only sets the baseline

  // 1 m forward (quantised to whole ticks: the delta is exact in ticks).
  const int32_t ticks = static_cast<int32_t>(1.0 / cfg.metersPerTick());
  const Pose2D p = odom.update(ticks, ticks);
  EXPECT_NEAR(p.x, 1.0 + ticks * cfg.metersPerTick(), 1e-9);
  EXPECT_NEAR(p.y, 2.0, 1e-9);
  EXPECT_NEAR(p.theta, 0.0, 1e-9);
}

TEST(Odometry, RotateInPlace) {
  RobotConfig cfg;
  Odometry odom(cfg);
  odom.reset(Pose2D(0, 0, 0));
  odom.update(0, 0);   // baseline

  // Right wheel forward, left backward -> CCW yaw of
  // (dr - dl)/track. Make dr - dl = track * (pi/2).
  const double want = kPi / 2.0;
  const double ds = 0.5 * want * cfg.trackWidthM;  // each wheel travels ds
  const int32_t dticks = static_cast<int32_t>(ds / cfg.metersPerTick());
  const Pose2D p = odom.update(-dticks, dticks);
  EXPECT_NEAR(p.theta, want, 0.01);          // tick quantisation
  EXPECT_NEAR(p.x, 0.0, 0.01);
  EXPECT_NEAR(p.y, 0.0, 0.01);
}

TEST(Odometry, RebootJumpIsRebaselined) {
  RobotConfig cfg;
  Odometry odom(cfg);
  odom.reset(Pose2D(0, 0, 0));
  odom.update(0, 0);
  // Drive to 1000 ticks in realistic 20 ms chunks (a single 1000-tick step
  // would itself trip the >1 m guard, which is the point of the guard).
  for (int32_t t = 10; t <= 1000; t += 10)
    odom.update(t, t);
  EXPECT_NEAR(odom.pose().x, 1000 * cfg.metersPerTick(), 1e-9);

  // Board rebooted: counters restart near 0 -> delta of ~-1000 ticks would
  // read as a huge backward jump; it must be ignored, not integrated.
  const Pose2D p = odom.update(5, 5);
  EXPECT_NEAR(p.x, 1000 * cfg.metersPerTick(), 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-9);

  // ...and motion resumes cleanly from the new baseline.
  const int32_t step = static_cast<int32_t>(0.1 / cfg.metersPerTick());
  const Pose2D q = odom.update(5 + step, 5 + step);
  EXPECT_NEAR(q.x, 1000 * cfg.metersPerTick() + step * cfg.metersPerTick(),
              1e-9);
}

TEST(Odometry, FirstSampleOnlyBaselines) {
  Odometry odom(RobotConfig{});
  odom.reset(Pose2D(3, 4, 1));
  const Pose2D p = odom.update(500, 500);
  EXPECT_NEAR(p.x, 3.0, 1e-12);
  EXPECT_NEAR(p.y, 4.0, 1e-12);
}

}  // namespace
}  // namespace vacuum
