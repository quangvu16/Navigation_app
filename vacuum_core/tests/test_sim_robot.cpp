// test_sim_robot.cpp — deterministic dynamics, odometry, lidar timing.

#include <gtest/gtest.h>

#include "sim/sim_robot.h"

namespace vacuum {
namespace {

SimNoise quiet() {
  SimNoise n;
  n.enabled = false;  // ideal physics: no slip, no noise, no motor lag
  return n;
}

SimRobot makeQuietRobot(SimWorld world) {
  RobotConfig cfg;
  return SimRobot(cfg, std::move(world), quiet(), 42);
}

// Runs the sim at 200 Hz for `seconds` with the given wheel RPM.
template <typename Fn>
void run(SimRobot& r, double seconds, Fn&& command) {
  const double dt = 0.005;
  const int steps = static_cast<int>(seconds / dt);
  for (int i = 0; i < steps; ++i) {
    command(r, i * dt);
    r.step(dt);
  }
}

TEST(SimRobot, DrivesStraightWithNoiseDisabled) {
  SimRobot r = makeQuietRobot(SimWorld::emptyRoom(20.0));
  r.setPose(Pose2D(2.0, 2.0, 0.0));
  // 30 rpm on 0.09615 m wheel: v = 30 * 2*pi*r/60 = 0.3021 m/s.
  const double rpm = 30.0;
  const double v = rpm * 2.0 * kPi * 0.09615 / 60.0;
  run(r, 2.0, [&](SimRobot& ro, double) { ro.setWheelRpm(rpm, rpm); });

  EXPECT_NEAR(r.sensors().truePose.x - 2.0, v * 2.0, 0.01);
  EXPECT_NEAR(r.sensors().truePose.y, 2.0, 1e-6);       // no slip noise
  EXPECT_NEAR(r.sensors().truePose.theta, 0.0, 1e-6);
  // Odometry must agree exactly when physics are quiet.
  EXPECT_NEAR(r.sensors().odomPose.x - 2.0, v * 2.0, 0.01);
  EXPECT_NEAR(r.sensors().odomPose.y, 2.0, 0.01);
  // Ticks: distance / metersPerTick.
  const double expectTicks = (v * 2.0) / (2.0 * kPi * 0.09615 / 90.0);
  EXPECT_NEAR(static_cast<double>(r.sensors().ticksL), expectTicks, 2.0);
  EXPECT_EQ(r.sensors().ticksL, r.sensors().ticksR);
}

TEST(SimRobot, RotatesInPlace) {
  SimRobot r = makeQuietRobot(SimWorld::emptyRoom(20.0));
  r.setPose(Pose2D(2.0, 2.0, 0.0));
  // Differential drive: +RPM left, -RPM right = CCW spin.
  const double rpm = 20.0;
  const double vWheel = rpm * 2.0 * kPi * 0.09615 / 60.0;
  const double omega = 2.0 * vWheel / 0.18;  // (vr - vl)/track
  run(r, 1.0, [&](SimRobot& ro, double) { ro.setWheelRpm(-rpm, rpm); });

  EXPECT_NEAR(r.sensors().truePose.theta, omega * 1.0, 0.02);
  EXPECT_NEAR(r.sensors().truePose.x, 2.0, 1e-6);
  EXPECT_NEAR(r.sensors().truePose.y, 2.0, 1e-6);
  EXPECT_NEAR(r.sensors().gyroZ, omega, 0.01);
}

TEST(SimRobot, LidarScansArriveAtSixHz) {
  SimRobot r = makeQuietRobot(SimWorld::emptyRoom(4.0));
  r.setPose(Pose2D(2.0, 2.0, 0.0));
  int scans = 0;
  double lastStart = -1;
  // 1.05 s: the 6th wrap completes exactly at t=1.0, one step before the
  // observer would sample it. 7 would need 1.166 s.
  run(r, 1.05, [&](SimRobot& ro, double) {
    ro.setWheelRpm(0.0, 0.0);
    if (ro.sensors().scanFresh) {
      ++scans;
      EXPECT_GT(ro.sensors().scanEndS, ro.sensors().scanStartS);
      if (lastStart >= 0) {
        EXPECT_NEAR(ro.sensors().scanStartS - lastStart, 1.0 / 6.0, 0.02);
      }
      lastStart = ro.sensors().scanStartS;
    }
  });
  EXPECT_EQ(scans, 6);
}

TEST(SimRobot, LidarRangesSeeTheRoom) {
  SimRobot r = makeQuietRobot(SimWorld::emptyRoom(4.0));
  r.setPose(Pose2D(2.0, 2.0, 0.0));
  run(r, 0.5, [&](SimRobot& ro, double) { ro.setWheelRpm(0.0, 0.0); });
  // Every beam should see a wall between 1.5 and 2.5 m from the centre
  // (corner directions up to 2.7 m).
  int valid = 0;
  for (float range : r.sensors().ranges) {
    if (range > 0.0f) {
      ++valid;
      EXPECT_GT(range, 1.5f);
      EXPECT_LT(range, 2.9f);
    }
  }
  EXPECT_GT(valid, 340);  // nearly every bin gets a return in a closed room
}

TEST(SimRobot, SonarSeesFrontWall) {
  SimRobot r = makeQuietRobot(SimWorld::emptyRoom(4.0));
  r.setPose(Pose2D(2.0, 2.0, 0.0));
  run(r, 0.5, [&](SimRobot& ro, double) { ro.setWheelRpm(0.0, 0.0); });
  EXPECT_NEAR(r.sensors().sonarM[0], 1.9, 0.05);  // front
  EXPECT_NEAR(r.sensors().sonarM[1], 1.9, 0.05);  // left
  EXPECT_NEAR(r.sensors().sonarM[2], 1.9, 0.05);  // right
}

TEST(SimRobot, NoisyModeStillTracksCommand) {
  RobotConfig cfg;
  SimWorld world = SimWorld::emptyRoom(20.0);
  SimRobot r(cfg, std::move(world), SimNoise(), 7);  // noise on by default
  r.setPose(Pose2D(2.0, 2.0, 0.0));
  const double rpm = 30.0;
  const double v = rpm * 2.0 * kPi * 0.09615 / 60.0;
  run(r, 2.0, [&](SimRobot& ro, double) { ro.setWheelRpm(rpm, rpm); });
  // The motor lag (tau=0.12 s) alone costs ~6% of the ideal distance over
  // 2 s; noise and slip add a little more. 15% band keeps the check meaningful
  // without being flaky.
  EXPECT_NEAR(r.sensors().truePose.x - 2.0, v * 2.0, v * 2.0 * 0.15);
  EXPECT_GT(std::abs(r.sensors().odomPose.x - r.sensors().truePose.x), 0.0);
}

}  // namespace
}  // namespace vacuum
