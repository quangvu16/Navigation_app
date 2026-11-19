// test_localization.cpp - Phase 4: likelihood field + AMCL.

#include <gtest/gtest.h>

#include <random>

#include "vacuum/amcl.h"
#include "vacuum/occupancy_grid.h"

namespace vacuum {
namespace {

// Paint-free test room: 6 x 4 m with a divider (gap at the top), interior
// filled free (see test_planning.cpp for why painting beats raycasting).
OccupancyGrid testRoom() {
  OccupancyGrid g(0.05);
  const int W = static_cast<int>(std::ceil(6.5 / 0.05));
  const int H = static_cast<int>(std::ceil(4.5 / 0.05));
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) g.setLogOdds(x, y, -1.0f);
  auto paint = [&](double x0, double y0, double x1, double y1) {
    const double len = std::hypot(x1 - x0, y1 - y0);
    const int steps = std::max(2, static_cast<int>(len / 0.025));
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      int gx, gy;
      g.worldToCell(x0 + t * (x1 - x0), y0 + t * (y1 - y0), gx, gy);
      g.setLogOdds(gx, gy, OccupancyGrid::kLOddMax);
    }
  };
  paint(0, 0, 6, 0); paint(6, 0, 6, 4);
  paint(6, 4, 0, 4); paint(0, 4, 0, 0);
  paint(2, 0, 2, 3);   // divider at x=2 - ASYMMETRIC rooms (a centred
  // divider would make the two rooms mirror images and the scan
  // likelihood ambiguous; odometry alone could not break the tie)
  return g;
}

// Scan from a pose against the map: base-frame points (like the sim does).
std::vector<Vec2> scanFrom(const OccupancyGrid& g, const Pose2D& pose) {
  std::vector<Vec2> pts;
  for (int i = 0; i < 360; i += 4) {
    const double ang = pose.theta + kPi - i * (2.0 * kPi / 360.0);
    double r = 0.05;
    while (r < 8.0) {
      const Vec2 w = pose.translation() + Vec2(std::cos(ang), std::sin(ang)) * r;
      int gx, gy;
      g.worldToCell(w.x, w.y, gx, gy);
      if (g.occupied(gx, gy)) break;
      r += 0.05;
    }
    if (r < 8.0)
      pts.push_back(rotate(Vec2(std::cos(ang), std::sin(ang)) * r, -pose.theta));
  }
  return pts;
}

TEST(LikelihoodField, DistanceToWall) {
  const OccupancyGrid g = testRoom();
  LikelihoodField f;
  f.build(g);
  // Interior point: distance to the nearest wall (divider at x=2 now).
  EXPECT_NEAR(f.distance(2.5, 1.5), 0.5, 0.06);
  // On a wall: ~0.
  EXPECT_LT(f.distance(2.0, 1.5), 0.06);
}

TEST(Mcl, ConvergesOnKnownMap) {
  const OccupancyGrid g = testRoom();
  LikelihoodField field;
  field.build(g);

  Mcl mcl;
  std::mt19937 rng(7);
  mcl.initGlobal(g, rng);
  ASSERT_TRUE(mcl.initialized());

  // Walk a short path in the left room. The trajectory must be
  // holonomically consistent with the odom motion model (unicycle arc):
  // rotate in place, THEN drive straight. Interpolating position in a
  // straight line while yawing independently (the obvious way) describes
  // a path no differential drive can execute, and the arc-based predict
  // diverges from it by ~0.1 m per turning step - the filter then loses
  // the robot while the "truth" slides away.
  std::mt19937 noise(11);
  std::normal_distribution<double> n(0.0, 0.005);
  Pose2D truth(1.0, 1.0, 0.0);
  Pose2D prevTruth = truth;
  const Pose2D waypoints[] = {{1.5, 1.0, 0.0}, {1.5, 2.0, kPi / 2},
                              {1.0, 2.0, kPi}, {1.0, 1.0, -kPi / 2}};
  auto stepOnce = [&](const Pose2D& target, bool rotatePhase) {
    if (rotatePhase) {
      // Pure rotation about the robot centre (dr = 0).
      const double dth =
          normalizeAngle(target.theta - truth.theta) / 10.0;
      prevTruth = truth;
      truth.theta = normalizeAngle(truth.theta + dth);
    } else {
      // Pure translation along the current heading (dtheta = 0).
      const double dist = std::hypot(target.x - truth.x, target.y - truth.y);
      const double dr = dist / 20.0;
      prevTruth = truth;
      truth.x += dr * std::cos(truth.theta);
      truth.y += dr * std::sin(truth.theta);
    }
    const Pose2D delta = Pose2D::relative(prevTruth, truth);
    mcl.predict(Pose2D(delta.x + n(noise), delta.y + n(noise),
                       delta.theta + n(noise)), rng);
    mcl.update(field, scanFrom(g, truth), 0.10, &g);
  };
  for (const Pose2D& wp : waypoints) {
    for (int s = 0; s < 10; ++s) stepOnce(wp, /*rotatePhase=*/true);
    for (int s = 0; s < 20; ++s) stepOnce(wp, /*rotatePhase=*/false);
  }

  const Pose2D est = mcl.pose();
  EXPECT_NEAR(est.x, truth.x, 0.20) << "MCL x";
  EXPECT_NEAR(est.y, truth.y, 0.20) << "MCL y";
  EXPECT_NEAR(normalizeAngle(est.theta - truth.theta), 0.0,
              10.0 * kPi / 180.0) << "MCL yaw";
  EXPECT_FALSE(mcl.looksLost());
}

TEST(Mcl, KidnapTriggersGlobalRecovery) {
  const OccupancyGrid g = testRoom();
  LikelihoodField field;
  field.build(g);
  Mcl mcl;
  std::mt19937 rng(7);
  mcl.initAround(Pose2D(1.5, 1.0, 0.0), rng);

  // Converge in place.
  const auto pts0 = scanFrom(g, Pose2D(1.5, 1.0, 0.0));
  for (int i = 0; i < 5; ++i) mcl.update(field, pts0, 0.10, &g);
  EXPECT_FALSE(mcl.looksLost());

  // Kidnap: teleport deep into the other room (the old cloud then scores
  // clearly below the loss threshold). Recovery afterwards must break the
  // rectangle's 180-degree scan symmetry - tempering keeps both the true
  // pose and its alias alive, and the post-reinit motion discriminates
  // them (static scans alone cannot).
  const Pose2D kidnapped(4.5, 1.0, kPi);
  const auto pts1 = scanFrom(g, kidnapped);
  bool lost = false;
  for (int i = 0; i < 10 && !lost; ++i) {
    mcl.update(field, pts1, 0.10, &g);
    lost = mcl.looksLost();
  }
  EXPECT_TRUE(lost) << "weight collapse must be detectable";

  // Recovery: global re-init, then converge - WHILE MOVING. A rectangular
  // room is invariant under 180-degree rotation, so a single static scan
  // cannot distinguish (x, y, theta) from its central reflection; odometry
  // motion is what breaks the alias (exactly as in real AMCL). Drive the
  // kidnapped robot forward and check the estimate against its FINAL pose.
  mcl.initGlobal(g, rng);
  Pose2D truth = kidnapped;
  Pose2D prev = truth;
  std::mt19937 noise(11);
  std::normal_distribution<double> n(0.0, 0.005);
  for (int i = 0; i < 25; ++i) {
    // Pure translation along the current heading (holonomic-consistent).
    prev = truth;
    truth.x += 0.03 * std::cos(truth.theta);
    truth.y += 0.03 * std::sin(truth.theta);
    const Pose2D delta = Pose2D::relative(prev, truth);
    mcl.predict(Pose2D(delta.x + n(noise), delta.y + n(noise),
                       delta.theta + n(noise)), rng);
    mcl.update(field, scanFrom(g, truth), 0.10, &g);
  }
  const Pose2D est = mcl.pose();
  // The right room is a rectangle: (x, y, theta) and its central
  // reflection through the room centre (4, 2) produce identical scans AND
  // track identically under the same odom deltas - both are equally valid
  // localizations and no likelihood can prefer one (only an asymmetric
  // cue like a doorway in view breaks the tie). Accept either mode.
  const Pose2D alt(8.0 - truth.x, 4.0 - truth.y,
                   normalizeAngle(truth.theta + kPi));
  const double dTruth = std::hypot(est.x - truth.x, est.y - truth.y);
  const double dAlt = std::hypot(est.x - alt.x, est.y - alt.y);
  EXPECT_LT(std::min(dTruth, dAlt), 0.30)
      << "est (" << est.x << "," << est.y << ") is neither the true pose "
      << "nor its symmetry image";
  const double yawTarget = dTruth <= dAlt ? truth.theta : alt.theta;
  EXPECT_NEAR(normalizeAngle(est.theta - yawTarget), 0.0,
              15.0 * kPi / 180.0);
}

}  // namespace
}  // namespace vacuum
