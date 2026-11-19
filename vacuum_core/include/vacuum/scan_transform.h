// scan_transform.h — the ONLY place that converts raw lidar bins into robot
// or world geometry.
//
// Wire convention (verified in firmware lidar_ld14.h): bin i is
// `i * (360/bins)` degrees CLOCKWISE from the lidar's zero mark. The bins are
// never reordered anywhere. The correction from sensor to base frame is:
//
//   angle_in_base = mirror ? -(i * step) : +(i * step)      (step = 2*pi/bins)
//   world_angle   = base.theta + lidarMountYaw + angle_in_base
//
// - `lidarMountYaw` places the zero mark relative to base +x (repo ROS setup:
//   angle0_deg = 180).
// - `lidarMirror` flips handedness for a mirrored/upside-down mounting (repo
//   ROS setup: flip_lr = true, i.e. the sensor's clockwise sweep stays
//   clockwise in the base frame: mirror = true -> angle = -i*step).
//
// Keeping this math in one tested header is why the decoder, the renderer,
// the scan matcher and the sim can never disagree about beam directions.

#pragma once

#include <cmath>

#include "vacuum/math.h"
#include "vacuum/robot_config.h"

namespace vacuum {

// Direction of bin `i` in the BASE frame (radians, CCW from base +x).
inline double binToBaseAngle(const RobotConfig& cfg, int bin, int bins) {
  const double step = 2.0 * kPi / bins;
  const double a = bin * step;
  return cfg.lidarMirror ? -a : a;
}

// Direction of bin `i` in the WORLD frame, given the base pose. Normalized
// to (-pi, pi] so headings compare directly.
inline double binToWorldAngle(const RobotConfig& cfg, const Pose2D& base,
                              int bin, int bins) {
  return normalizeAngle(base.theta + deg2rad(cfg.lidarMountYawDeg) +
                        binToBaseAngle(cfg, bin, bins));
}

// Laser origin in the world frame (lidar offset in base frame).
inline Vec2 laserOrigin(const RobotConfig& cfg, const Pose2D& base) {
  return base.translation() +
         rotate(Vec2(cfg.lidarOffsetXM, cfg.lidarOffsetYM), base.theta);
}

// World point measured by bin `i` (range in metres). Zero/no-return ranges
// must be filtered by the caller (range <= 0).
inline Vec2 binToWorldPoint(const RobotConfig& cfg, const Pose2D& base,
                            int bin, int bins, double rangeM) {
  const double ang = binToWorldAngle(cfg, base, bin, bins);
  return laserOrigin(cfg, base) + Vec2(std::cos(ang), std::sin(ang)) * rangeM;
}

}  // namespace vacuum
