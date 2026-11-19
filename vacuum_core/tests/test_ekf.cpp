// test_ekf.cpp - the Phase-2 acceptance: drive a 2 m square with realistic
// wheel calibration error + gyro bias/noise; the EKF must close within
// 5 cm / 3 deg (prompt §8) and beat raw odometry.

#include <gtest/gtest.h>

#include <random>

#include "vacuum/ekf.h"
#include "vacuum/odometry.h"

namespace vacuum {
namespace {

constexpr double kDt = 0.02;          // 50 Hz, matches HBT2/IMU cadence
constexpr double kBias = 0.02;        // rad/s injected gyro bias
constexpr double kVScale = 0.015;     // encoders read 1.5 % high
constexpr double kWScale = 0.03;      // ...and 3 % high on yaw

struct SquareRun {
  Pose2D truth;
  Pose2D ekfPose;
  Pose2D odomPose;
  double bias = 0.0;
};

// 4 sides of side_m, 90 deg turns, after stillSecs of standstill (the real
// robot is still while the app starts a run - that is the bias window).
SquareRun runSquare(double side_m, double stillSecs, uint32_t seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> gyroNoise(0.0, 0.005);

  RobotConfig cfg;
  Odometry odom(cfg);
  Ekf ekf;
  odom.reset(Pose2D());
  ekf.reset(Pose2D());

  int32_t ticksL = 0, ticksR = 0;
  const double mpt = cfg.metersPerTick();
  Pose2D truth;   // ground truth of this synthetic run

  auto step = [&](double v_cmd, double w_cmd) {
    // Ground truth motion (what the robot actually does).
    const double th_mid = truth.theta + 0.5 * w_cmd * kDt;
    truth.x += v_cmd * std::cos(th_mid) * kDt;
    truth.y += v_cmd * std::sin(th_mid) * kDt;
    truth.theta = normalizeAngle(truth.theta + w_cmd * kDt);

    // What the wheels REPORT: scale error (calibration), same slip both
    // wheels so it shows up as scale, not turn noise.
    const double v_meas = v_cmd * (1.0 + kVScale);
    const double w_meas = w_cmd * (1.0 + kWScale);

    // Ticks are per-wheel: v +/- w*track/2.
    const double vl = v_meas - w_meas * cfg.trackWidthM * 0.5;
    const double vr = v_meas + w_meas * cfg.trackWidthM * 0.5;
    ticksL += static_cast<int32_t>(std::lround(vl * kDt / mpt));
    ticksR += static_cast<int32_t>(std::lround(vr * kDt / mpt));
    odom.update(ticksL, ticksR);

    const double gyro = w_cmd + kBias + gyroNoise(rng);
    ekf.predict(kDt, v_meas, w_meas);
    ekf.updateGyro(gyro);
  };

  // Standstill: bias window.
  const int stillSteps = static_cast<int>(stillSecs / kDt);
  for (int i = 0; i < stillSteps; ++i) step(0.0, 0.0);

  // The square: forward, turn 90, x4.
  const double v = 0.25, w = 0.8;
  const double driveT = side_m / v;
  const double turnT = (kPi / 2.0) / w;
  for (int leg = 0; leg < 4; ++leg) {
    for (double t = 0; t < driveT; t += kDt) step(v, 0.0);
    for (double t = 0; t < turnT; t += kDt) step(0.0, w);
  }

  SquareRun r;
  r.truth = truth;
  r.ekfPose = ekf.pose();
  r.odomPose = odom.pose();
  r.bias = ekf.gyroBias();
  return r;
}

TEST(Ekf, GyroBiasConvergesWhileStationary) {
  const SquareRun r = runSquare(0.5, 3.0, 7);
  EXPECT_NEAR(r.bias, kBias, 0.004) << "3 s still must estimate the bias";
}

TEST(Ekf, SquareClosesWithinTolerance) {
  // Phase-2 acceptance (prompt §8): 2 m square -> within 5 cm / 3 deg.
  const SquareRun r = runSquare(0.5, 3.0, 42);

  const double ekfPosErr = std::hypot(r.ekfPose.x - r.truth.x,
                                      r.ekfPose.y - r.truth.y);
  const double ekfYawErr = std::abs(normalizeAngle(r.ekfPose.theta - r.truth.theta));
  EXPECT_LT(ekfPosErr, 0.05) << "EKF position must close within 5 cm";
  EXPECT_LT(ekfYawErr, 3.0 * kPi / 180.0) << "EKF yaw must close within 3 deg";
}

TEST(Ekf, BeatsRawOdometry) {
  const SquareRun r = runSquare(0.5, 3.0, 42);

  const double odomPosErr = std::hypot(r.odomPose.x - r.truth.x,
                                       r.odomPose.y - r.truth.y);
  const double ekfPosErr = std::hypot(r.ekfPose.x - r.truth.x,
                                      r.ekfPose.y - r.truth.y);
  const double odomYawErr = std::abs(normalizeAngle(r.odomPose.theta - r.truth.theta));
  const double ekfYawErr = std::abs(normalizeAngle(r.ekfPose.theta - r.truth.theta));

  // The wheel scale error alone walks the odom heading by 3 % of 360 deg
  // (~10.8 deg), which drags the position off by more than the EKF's.
  EXPECT_GT(odomYawErr, 0.05) << "test must be meaningful: odom drifts";
  EXPECT_LT(ekfYawErr, odomYawErr) << "gyro fusion must beat raw odom yaw";
  EXPECT_LT(ekfPosErr, odomPosErr) << "and therefore position too";
}

TEST(Ekf, GyroRecoversFromWheelSlipDuringTurn) {
  // One 90 deg turn where the wheels UNDER-REPORT yaw by 30 % (carpet slip):
  // odometry ends ~27 deg short; the gyro sees the true rate and the EKF
  // must drag the heading back.
  RobotConfig cfg;
  Odometry odom(cfg);
  Ekf ekf;
  odom.reset(Pose2D());
  ekf.reset(Pose2D());

  int32_t ticksL = 0, ticksR = 0;
  const double mpt = cfg.metersPerTick();
  const double w_true = 0.8, w_slip = w_true * 0.7;
  const double turnT = (kPi / 2.0) / w_true;
  std::mt19937 rng(11);
  std::normal_distribution<double> noise(0.0, 0.003);

  for (double t = 0; t < turnT; t += kDt) {
    const double vl = -w_slip * cfg.trackWidthM * 0.5;
    const double vr = +w_slip * cfg.trackWidthM * 0.5;
    ticksL += static_cast<int32_t>(std::lround(vl * kDt / mpt));
    ticksR += static_cast<int32_t>(std::lround(vr * kDt / mpt));
    odom.update(ticksL, ticksR);
    ekf.predict(kDt, 0.0, w_slip);
    ekf.updateGyro(w_true + noise(rng));   // gyro is honest
  }

  const double odomYaw = std::abs(normalizeAngle(odom.pose().theta - kPi / 2));
  const double ekfYaw = std::abs(normalizeAngle(ekf.pose().theta - kPi / 2));
  EXPECT_GT(odomYaw, 15.0 * kPi / 180.0) << "slipping odom must be badly off";
  EXPECT_LT(ekfYaw, 7.0 * kPi / 180.0) << "gyro must recover most of it";
  EXPECT_LT(ekfYaw, odomYaw);
}

}  // namespace
}  // namespace vacuum
