// ekf.h - 2D error-state-lite EKF, state [x, y, θ, v, ω] (prompt §5).
//
// Prediction uses the WHEEL-derived control (v, ω) exactly as the prompt
// specifies; the measurement update fuses the gyro Z (bias-corrected).
// Why the gyro helps the pose at all: the control noise on ω_w enters both
// θ' (via G(2,1) = dt) and the ω state (via G(4,1) = 1), so
// Cov(θ, ω) = dt·var(ω_w) ≠ 0 and correcting ω with the gyro drags θ along.
// (Correlating them any other way is guesswork; this is the actual shared
// noise source - both come from the same wheel slip.)
//
// Gyro bias: estimated only while the wheels say the robot is stationary
// for >= stillSecs (2 s, prompt §5) - a moving robot's gyro mean is not a
// bias, it is the turn rate. While moving the last estimate is frozen.
//
// Why the magnetometer is never used: hub-motor magnets corrupt it
// (mpu9250.h / prompt §2).
//
// Frames: this fuses odom-frame motion; map->odom corrections come from
// scan matching (Phase 3) as pose updates. All SI.

#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "vacuum/math.h"
#include "vacuum/robot_config.h"

namespace vacuum {

class Ekf {
 public:
  using Vec5 = Eigen::Matrix<double, 5, 1>;
  using Mat5 = Eigen::Matrix<double, 5, 5>;

  struct Params {
    // Wheel control noise (the odom source): slip on carpet dominates, so
    // these are deliberately larger than encoder resolution.
    double wheelVStd = 0.03;        // m/s
    double wheelWStd = 0.05;        // rad/s
    // Gyro white noise (MPU9250 at ±500 deg/s is ~0.01 rad/s RMS).
    double gyroStd = 0.01;          // rad/s
    // Numerical floor so P never collapses to zero (over-confident filters
    // stop correcting - the classic EKF failure).
    double qPos = 1e-8;             // m^2 / step
    double qTheta = 1e-8;           // rad^2 / step
    // Stationary gate for bias estimation (prompt §5: > 2 s still).
    double stillV = 0.01;           // m/s
    double stillW = 0.02;           // rad/s
    double stillSecs = 2.0;
    // Initial covariance (pose known from "set pose" / init otherwise).
    double p0Pos = 0.01;            // m^2
    double p0Theta = 0.05;          // rad^2  (~13 deg)
    double p0V = 0.01;              // (m/s)^2
    double p0W = 0.02;              // (rad/s)^2
  };

  // Two ctors, not one with `= Params()` default arg: a nested class's
  // default member initializers may not be used in the enclosing class's
  // default arguments (the enclosing class is incomplete there).
  Ekf() : p_() { reset(Pose2D()); }
  explicit Ekf(const Params& p) : p_(p) { reset(Pose2D()); }

  void reset(const Pose2D& pose) {
    x_.setZero();
    x_(0) = pose.x;
    x_(1) = pose.y;
    x_(2) = pose.theta;
    P_.setZero();
    P_(0, 0) = P_(1, 1) = p_.p0Pos;
    P_(2, 2) = p_.p0Theta;
    P_(3, 3) = p_.p0V;
    P_(4, 4) = p_.p0W;
    bias_ = 0.0;
    bias_mean_ = 0.0;
    bias_n_ = 0;
    stillFor_ = 0.0;
    last_vw_ = last_ww_ = 0.0;
  }

  // Prediction step: wheel-derived control over dt seconds.
  void predict(double dt, double wheel_v, double wheel_w) {
    if (dt <= 0.0) return;
    last_vw_ = wheel_v;
    last_ww_ = wheel_w;

    const double th = x_(2);
    const double c = std::cos(th), s = std::sin(th);

    // State transition (pose from the CONTROL, v/ω refreshed from wheels).
    Mat5 F = Mat5::Identity();
    F(0, 2) = -wheel_v * s * dt;
    F(1, 2) = wheel_v * c * dt;
    F(3, 3) = 0.0;   // v' = wheel value, independent of the old state
    F(4, 4) = 0.0;   // ω' likewise

    // Control-noise mapping: same G that couples θ and ω (see header note).
    Eigen::Matrix<double, 5, 2> G;
    G.setZero();
    G(0, 0) = c * dt;
    G(1, 0) = s * dt;
    G(2, 1) = dt;
    G(3, 0) = 1.0;
    G(4, 1) = 1.0;
    Eigen::Matrix2d Rw;
    Rw << p_.wheelVStd * p_.wheelVStd, 0.0,
          0.0, p_.wheelWStd * p_.wheelWStd;
    Mat5 Q = G * Rw * G.transpose();
    Q(0, 0) += p_.qPos;
    Q(1, 1) += p_.qPos;
    Q(2, 2) += p_.qTheta;

    // Integrate the pose with the wheels (midpoint on θ, as odometry.h).
    const double th_mid = th + 0.5 * wheel_w * dt;
    x_(0) += wheel_v * std::cos(th_mid) * dt;
    x_(1) += wheel_v * std::sin(th_mid) * dt;
    x_(2) = normalizeAngle(th + wheel_w * dt);
    x_(3) = wheel_v;
    x_(4) = wheel_w;

    P_ = F * P_ * F.transpose() + Q;

    // Stationary clock for bias estimation.
    if (std::abs(wheel_v) < p_.stillV && std::abs(wheel_w) < p_.stillW)
      stillFor_ += dt;
    else
      stillFor_ = 0.0;
  }

  // Gyro Z measurement (raw rad/s, includes bias - we subtract it here).
  // Call at the IMU rate (50 Hz).
  void updateGyro(double gyro_z_raw) {
    // Bias from an incremental mean of the still window (prompt §5: the
    // estimate only becomes valid after stillSecs of agreement from the
    // wheels). A mean over 2 s of 50 Hz samples has std ~0.001 rad/s -
    // an exponential track would still be 60 % short by then.
    if (stillFor_ > 0.0) {
      ++bias_n_;
      bias_mean_ += (gyro_z_raw - bias_mean_) / static_cast<double>(bias_n_);
      if (stillFor_ >= p_.stillSecs)
        bias_ = bias_mean_;   // frozen while moving (stillFor_ resets)
    } else {
      bias_n_ = 0;            // motion: the window is only valid while still
      bias_mean_ = 0.0;
    }

    const double z = gyro_z_raw - bias_;
    const double innov = z - x_(4);

    Eigen::Matrix<double, 5, 1> H;
    H.setZero();
    H(4) = 1.0;
    const double R = p_.gyroStd * p_.gyroStd;
    const double S = H.dot(P_ * H) + R;   // H P H^T for a single row
    const Eigen::Matrix<double, 5, 1> K = (P_ * H) / S;

    x_ += K * innov;
    x_(2) = normalizeAngle(x_(2));
    const Mat5 IKH = Mat5::Identity() - K * H.transpose();
    P_ = IKH * P_;   // standard form; tests assert innovations stay sane
                     // (a collapsed P would show up as divergence there)
  }

  const Pose2D& pose() const {
    pose_cache_ = Pose2D(x_(0), x_(1), x_(2));
    return pose_cache_;
  }
  double x() const { return x_(0); }
  double y() const { return x_(1); }
  double theta() const { return x_(2); }
  double v() const { return x_(3); }
  double omega() const { return x_(4); }
  double gyroBias() const { return bias_; }
  double stationaryFor() const { return stillFor_; }
  const Mat5& covariance() const { return P_; }

  // Pose update for future scan-match corrections (Phase 3): injects a
  // map-frame pose measurement with isotropic noise.
  void updatePose(double mx, double my, double mtheta, double std_xy,
                  double std_theta) {
    Eigen::Matrix<double, 3, 5> H3;
    H3.setZero();
    H3(0, 0) = 1.0;
    H3(1, 1) = 1.0;
    H3(2, 2) = 1.0;
    Eigen::Matrix<double, 3, 1> innov;
    innov << mx - x_(0), my - x_(1), normalizeAngle(mtheta - x_(2));
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R(0, 0) = R(1, 1) = std_xy * std_xy;
    R(2, 2) = std_theta * std_theta;
    const Eigen::Matrix<double, 3, 3> S = H3 * P_ * H3.transpose() + R;
    const Eigen::Matrix<double, 5, 3> K = P_ * H3.transpose() * S.inverse();
    x_.head<3>() += (K * innov).head<3>();
    x_(2) = normalizeAngle(x_(2));
    P_ = (Mat5::Identity() - K * H3) * P_;
  }

 private:
  Params p_;
  Vec5 x_ = Vec5::Zero();
  Mat5 P_ = Mat5::Zero();
  double bias_ = 0.0;
  double bias_mean_ = 0.0;
  long bias_n_ = 0;
  double stillFor_ = 0.0;
  double last_vw_ = 0.0, last_ww_ = 0.0;
  mutable Pose2D pose_cache_;
};

}  // namespace vacuum
