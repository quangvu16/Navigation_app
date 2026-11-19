// odometry.h - differential-drive odometry from cumulative wheel ticks.
//
// The wire convention (verified in hb_link.h): ticks are RUNNING TOTALS, not
// deltas, so a dropped datagram costs nothing - the next one still carries
// the truth. A jump larger than kMaxStepM in one update means the board
// rebooted (counters restarted at 0) or a datagram was lost across a long
// gap: re-baseline instead of integrating the jump (same policy as the
// firmware's HB_RESTART_GAP_MS, applied phone-side where the timestamps
// live).
//
// Frames: the pose this produces is odom-frame (drifts vs map). Feeding it
// into the EKF gives the fused estimate; see ekf.h.

#pragma once

#include <cstdint>

#include "vacuum/math.h"
#include "vacuum/robot_config.h"

namespace vacuum {

class Odometry {
 public:
  explicit Odometry(const RobotConfig& cfg = RobotConfig())
      : mpt_(cfg.metersPerTick()), track_(cfg.trackWidthM) {}

  void reset(const Pose2D& pose = Pose2D()) {
    pose_ = pose;
    have_last_ = false;
  }

  // Feed the current cumulative tick totals. Returns the pose after
  // integrating the delta (also stored).
  Pose2D update(int32_t ticks_l, int32_t ticks_r) {
    if (!have_last_) {
      last_l_ = ticks_l;
      last_r_ = ticks_r;
      have_last_ = true;
      return pose_;
    }
    const double dl = static_cast<double>(ticks_l - last_l_) * mpt_;
    const double dr = static_cast<double>(ticks_r - last_r_) * mpt_;
    last_l_ = ticks_l;
    last_r_ = ticks_r;

    // Sanity: > 1 m in one 20 ms step is impossible at 0.5 m/s (0.01 m);
    // treat it as a counter reset, not motion.
    if (std::abs(dl) > kMaxStepM || std::abs(dr) > kMaxStepM)
      return pose_;

    const double ds = 0.5 * (dl + dr);          // arc length
    const double dth = (dr - dl) / track_;      // yaw from the wheel pair
    // Midpoint integration: exact for arcs, no O(dt^2) straight-line drift
    // while turning.
    const double th_mid = pose_.theta + 0.5 * dth;
    pose_.x += ds * std::cos(th_mid);
    pose_.y += ds * std::sin(th_mid);
    pose_.theta = normalizeAngle(pose_.theta + dth);
    return pose_;
  }

  const Pose2D& pose() const { return pose_; }

 private:
  static constexpr double kMaxStepM = 1.0;  // per update; > any real motion
  double mpt_;
  double track_;
  Pose2D pose_;
  int32_t last_l_ = 0, last_r_ = 0;
  bool have_last_ = false;
};

}  // namespace vacuum
