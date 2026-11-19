// test_math.cpp — geometry primitives.

#include <gtest/gtest.h>

#include "vacuum/math.h"

namespace vacuum {
namespace {

TEST(Math, NormalizeAngleWrapsToMinusPiPi) {
  EXPECT_NEAR(normalizeAngle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(normalizeAngle(3.0 * kPi), kPi, 1e-9);
  EXPECT_NEAR(normalizeAngle(-3.0 * kPi), kPi, 1e-9);
  EXPECT_NEAR(normalizeAngle(2.0 * kPi + 0.1), 0.1, 1e-9);
  EXPECT_NEAR(normalizeAngle(-0.1 - 2.0 * kPi), -0.1, 1e-9);
}

TEST(Math, RotatePreservesNorm) {
  const Vec2 v(0.3, -1.2);
  const Vec2 r = rotate(v, 0.77);
  EXPECT_NEAR(r.norm(), v.norm(), 1e-12);
  EXPECT_NEAR(r.dot(v), v.normSquared() * std::cos(0.77), 1e-9);
}

TEST(Math, PoseComposeInverseRoundTrip) {
  const Pose2D a(1.0, 2.0, 0.5);
  const Pose2D b(-0.3, 0.7, -1.1);
  const Pose2D ab = a.compose(b);
  const Pose2D back = a.inverse().compose(ab);  // should equal b
  EXPECT_NEAR(back.x, b.x, 1e-9);
  EXPECT_NEAR(back.y, b.y, 1e-9);
  EXPECT_NEAR(back.theta, b.theta, 1e-9);

  const Pose2D identity = a.compose(a.inverse());
  EXPECT_NEAR(identity.x, 0.0, 1e-9);
  EXPECT_NEAR(identity.y, 0.0, 1e-9);
  EXPECT_NEAR(identity.theta, 0.0, 1e-9);
}

TEST(Math, RelativePose) {
  const Pose2D from(1.0, 0.0, kPi / 2.0);  // at +x, facing +y
  const Pose2D to(1.0, 1.0, kPi / 2.0);    // 1 m "ahead" of from
  const Pose2D rel = Pose2D::relative(from, to);
  EXPECT_NEAR(rel.x, 1.0, 1e-9);   // forward in from's frame
  EXPECT_NEAR(rel.y, 0.0, 1e-9);
  EXPECT_NEAR(rel.theta, 0.0, 1e-9);
}

}  // namespace
}  // namespace vacuum
