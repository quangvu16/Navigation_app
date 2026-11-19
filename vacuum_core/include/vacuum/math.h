// math.h — 2D geometry primitives shared by every part of vacuum_core.
//
// Frames (fixed, see docs/ARCHITECTURE.md):
//   map  -> odom -> base -> laser
//   map/odom: x right, y up (math convention). Screen rendering flips Y.
//   base: robot centre, x forward, y left. theta CCW-positive.
// Angles are radians unless a field name says Deg. All in SI (metres, sec).

#pragma once

#include <cmath>

namespace vacuum {

constexpr double kPi = 3.14159265358979323846;

// Wrap to (-pi, pi]. Chosen over [-pi, pi) so that a robot facing exactly
// "backwards" reports +pi deterministically in UI and tests.
inline double normalizeAngle(double a) {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a <= 0.0) a += 2.0 * kPi;
  return a - kPi;
}

struct Vec2 {
  double x = 0.0, y = 0.0;

  Vec2() = default;
  Vec2(double x_, double y_) : x(x_), y(y_) {}

  Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  Vec2 operator*(double s) const { return {x * s, y * s}; }
  Vec2 operator-() const { return {-x, -y}; }

  double dot(const Vec2& o) const { return x * o.x + y * o.y; }
  double norm() const { return std::sqrt(x * x + y * y); }
  double normSquared() const { return x * x + y * y; }
};

inline Vec2 rotate(const Vec2& v, double th) {
  const double c = std::cos(th), s = std::sin(th);
  return {c * v.x - s * v.y, s * v.x + c * v.y};
}

// Rigid transform (x, y, theta).
struct Pose2D {
  double x = 0.0, y = 0.0, theta = 0.0;

  Pose2D() = default;
  Pose2D(double x_, double y_, double th_) : x(x_), y(y_), theta(normalizeAngle(th_)) {}

  Vec2 translation() const { return {x, y}; }

  // a ⊕ b: apply b in a's frame.
  Pose2D compose(const Pose2D& b) const {
    const Vec2 t = translation() + rotate(b.translation(), theta);
    return {t.x, t.y, theta + b.theta};
  }

  Pose2D inverse() const {
    const Vec2 t = rotate(translation(), -theta);
    return {-t.x, -t.y, -theta};
  }

  // Pose of `to` expressed in `from`'s frame: from⁻¹ ⊕ to.
  static Pose2D relative(const Pose2D& from, const Pose2D& to) {
    return from.inverse().compose(to);
  }
};

inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }

}  // namespace vacuum
