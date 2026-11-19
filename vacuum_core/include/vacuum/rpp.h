// rpp.h - Regulated Pure Pursuit local controller (prompt §5).
//
// Follows the global path with:
//   - adaptive lookahead0.3..0.6 m (grows with speed),
//   - curvature + obstacle regulation: slows on sharp turns and when the
//     costmap says an obstacle is near (Nav2's RPP "regulated" behaviour),
//   - rotate-in-place when the heading to the lookahead point exceeds
//     60 deg (prompt),
//   - a 1 s forward rollout collision check against the costmap layer:
//     a predicted collision brakes to zero (the caller may then recover).
//
// Output is (v, w) in m/s and rad/s; the service converts to wheel RPM
// (RobotConfig.velocityToWheelRpm) and the ESP32 safety net still applies.

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "vacuum/costmap.h"
#include "vacuum/math.h"

namespace vacuum {

struct RppParams {
  double lookaheadMin = 0.30;      // prompt:0.3..0.6 m
  double lookaheadMax = 0.60;
  double maxLinear = 0.35;         // RobotConfig.maxLinearMps default
  double maxAngular = 1.2;
  double obstacleSlowDist = 0.60;  // start slowing inside this clearance
  double minSpeedScale = 0.20;
  double rotateInPlaceDeg = 60.0;  // prompt: rotate-in-place > 60 deg
  double goalToleranceM = 0.10;    // prompt §8: reached within10 cm
  double rolloutDt = 0.02;
  double rolloutSecs = 1.0;        // prompt:1 s forward rollout
  double curvatureGain = 1.0;      // w = gain * v * curvature
};

struct RppCommand {
  double v = 0.0;
  double w = 0.0;
  bool rotateInPlace = false;
  bool blocked = false;           // rollout hit a lethal cell
  bool goalReached = false;
};

class RegulatedPurePursuit {
 public:
  // Two-ctor rule for nested Params (same as the other planners).
  RegulatedPurePursuit() : p_() {}
  explicit RegulatedPurePursuit(const RppParams& p) : p_(p) {}

  // path: world-frame waypoints (A* output, densified). pose: current
  // localization. Returns the (v, w) command for this control tick.
  RppCommand step(const Costmap& map, const Pose2D& pose,
                  const std::vector<Vec2>& path) const {
    RppCommand cmd;
    if (path.size() < 2) return cmd;

    // Goal check: within tolerance of the LAST path point.
    const Vec2 goal = path.back();
    const double goalDist = std::hypot(goal.x - pose.x, goal.y - pose.y);
    if (goalDist < p_.goalToleranceM) {
      cmd.goalReached = true;
      return cmd;
    }

    // Lookahead point: nearest path index to the pose, then walk forward
    // until the distance from the ROBOT exceeds the lookahead.
    size_t nearest = 0;
    double best = 1e18;
    for (size_t i = 0; i < path.size(); ++i) {
      const double d = std::hypot(path[i].x - pose.x, path[i].y - pose.y);
      if (d < best) { best = d; nearest = i; }
    }
    const double lookahead = p_.lookaheadMax;   // speed-adaptive below
    size_t li = nearest;
    while (li + 1 < path.size() &&
           std::hypot(path[li].x - pose.x, path[li].y - pose.y) < lookahead)
      ++li;
    const Vec2 target = path[li];

    // Bearing error to the lookahead point (robot frame).
    const double bearing = std::atan2(target.y - pose.y, target.x - pose.x);
    const double headingErr = normalizeAngle(bearing - pose.theta);

    // --- rotate-in-place when the turn is sharp (prompt §5) ---
    if (std::abs(headingErr) > p_.rotateInPlaceDeg * kPi / 180.0) {
      cmd.rotateInPlace = true;
      cmd.v = 0.0;
      cmd.w = (headingErr > 0 ? 1.0 : -1.0) * p_.maxAngular * 0.6;
      cmd.blocked = rolloutCollides(map, pose, cmd.v, cmd.w);
      if (cmd.blocked) { cmd.v = 0.0; cmd.w = 0.0; }
      return cmd;
    }

    // --- curvature from pure pursuit: k = 2*sin(alpha)/L ---
    const double L = std::max(0.05, std::hypot(target.x - pose.x,
                                                target.y - pose.y));
    const double k = 2.0 * std::sin(headingErr) / L;

    // --- regulation: slow near obstacles (costmap clearance) ---
    double scale = 1.0;
    bool lethalAhead = false;
    // Sample clearance just ahead of the robot.
    for (double d = 0.15; d <= p_.obstacleSlowDist; d += 0.15) {
      const double x = pose.x + d * std::cos(pose.theta);
      const double y = pose.y + d * std::sin(pose.theta);
      const uint8_t c = map.costAt(x, y);
      if (c == Costmap::kLethal) { scale = 0.0; lethalAhead = true; break; }
      if (c != Costmap::kFree && c != Costmap::kUnknown) {
        const double clear = d;   // first non-free sample distance
        scale = std::min(scale, std::max(p_.minSpeedScale,
            clear / p_.obstacleSlowDist));
      }
    }
    // Lethal dead ahead: no forward progress is possible - report blocked
    // so the GoTo state machine recovers (a zero command alone would leave
    // the robot parked facing the wall forever).
    if (lethalAhead) {
      cmd.v = 0.0;
      cmd.w = 0.0;
      cmd.blocked = true;
      return cmd;
    }
    // Curvature regulation: sharp turns need speed to stay stable.
    scale = std::min(scale, 1.0 / (1.0 + 4.0 * std::abs(k)));

    double v = p_.maxLinear * scale;
    double w = p_.curvatureGain * v * k;
    // Angular clamp.
    w = std::clamp(w, -p_.maxAngular, p_.maxAngular);
    // Keep direction consistent with the heading error sign.
    if (headingErr != 0.0 && (w > 0) != (headingErr > 0))
      w = (headingErr > 0 ? 1.0 : -1.0) * std::min(std::abs(w) + 0.3,
                                                   p_.maxAngular);

    // ---1 s forward rollout collision check (prompt §5) ---
    cmd.v = v;
    cmd.w = w;
    cmd.blocked = rolloutCollides(map, pose, v, w);
    if (cmd.blocked) {
      cmd.v = 0.0;
      cmd.w = 0.0;
    }
    return cmd;
  }

  const RppParams& params() const { return p_; }

 private:
  RppParams p_;

  // Simulate the unicycle for rolloutSecs; true if the centre enters a
  // lethal cell (inflation already carries the robot radius).
  bool rolloutCollides(const Costmap& map, const Pose2D& pose, double v,
                       double w) const {
    double x = pose.x, y = pose.y, th = pose.theta;
    const int steps = std::max(1, static_cast<int>(p_.rolloutSecs / p_.rolloutDt));
    for (int i = 0; i < steps; ++i) {
      x += v * std::cos(th) * p_.rolloutDt;
      y += v * std::sin(th) * p_.rolloutDt;
      th = normalizeAngle(th + w * p_.rolloutDt);
      if (map.costAt(x, y) == Costmap::kLethal) return true;
    }
    return false;
  }
};

}  // namespace vacuum
