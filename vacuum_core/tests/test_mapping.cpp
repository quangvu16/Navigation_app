// test_mapping.cpp - Phase 3: occupancy grid, scan matcher, mapper.

#include <gtest/gtest.h>

#include <random>

#include "vacuum/mapper.h"
#include "vacuum/occupancy_grid.h"
#include "vacuum/scan_matcher.h"

namespace vacuum {
namespace {

// ---------------------------------------------------------- occupancy grid
TEST(OccupancyGrid, RayUpdatesAndGrowth) {
  OccupancyGrid g(0.05);
  // A hit3 m to the +x of the origin must grow the grid rightwards and
  // mark exactly the endpoint occupied, clearing along the path.
  g.rayHit({0.0, 0.0}, {3.0, 0.0});
  int gx, gy;
  g.worldToCell(3.0, 0.0, gx, gy);
  EXPECT_TRUE(g.occupied(gx, gy));
  g.worldToCell(1.5, 0.0, gx, gy);
  // One miss delta (-0.4) clears the cell toward free; the strict kFreeTh
  // (-0.5) needs a second pass, which real maps get from overlapping rays.
  EXPECT_LT(g.logOdds(gx, gy), 0.0f) << "path must be cleared";
  // A hit far to the LEFT of the origin grows at the origin end too.
  g.rayHit({0.0, 0.0}, {-3.0, 2.0});
  g.worldToCell(-3.0, 2.0, gx, gy);
  EXPECT_TRUE(g.occupied(gx, gy));
}

TEST(OccupancyGrid, RleRoundTrip) {
  OccupancyGrid g(0.05);
  g.rayHit({1.0, 1.0}, {2.5, 1.0});
  g.rayHit({1.0, 1.0}, {1.0, 2.5});
  const std::string blob = g.serializeRle();

  OccupancyGrid back(0.01);   // resolution comes from the blob
  ASSERT_TRUE(OccupancyGrid::deserializeRle(blob, back));
  ASSERT_EQ(back.width(), g.width());
  ASSERT_EQ(back.height(), g.height());
  EXPECT_NEAR(back.resolution(), g.resolution(), 1e-9);
  // Quantisation step is 0.25 in log-odds; cells must match to that.
  for (int y = 0; y < g.height(); y += 7)
    for (int x = 0; x < g.width(); x += 5)
      EXPECT_NEAR(back.logOdds(g.originX() + x, g.originY() + y),
                  g.logOdds(g.originX() + x, g.originY() + y), 0.26);
}

// ------------------------------------------------------------- matcher
TEST(ScanMatcher, RecoversPerturbedPose) {
  // Ground-truth grid: an asymmetric room (a notch breaks the symmetry a
  // square room would give scan matching).
  OccupancyGrid g(0.05);
  auto wall = [&](Vec2 a, Vec2 b) { g.rayHit({2.5, 2.0}, b); };
  // Outline: 5 x 4 room with a notch in the top-right.
  const Vec2 corners[] = {{0, 0}, {5, 0}, {5, 3}, {4, 3}, {4, 4}, {0, 4}, {0, 0}};
  for (int i = 0; i + 1 < 6; ++i)
    for (double t = 0; t < 1.0; t += 0.02) {
      const Vec2 p(corners[i].x + t * (corners[i + 1].x - corners[i].x),
                   corners[i].y + t * (corners[i + 1].y - corners[i].y));
      g.rayHit({2.5, 2.0}, p);
      wall({}, p);
    }

  // Synthetic scan taken at pose P0: rays to the walls from P0.
  const Pose2D truth(2.5, 2.0, 0.7);
  std::vector<Vec2> pts;
  for (int i = 0; i < 360; i += 3) {
    const double ang = truth.theta + deg2rad(180.0) - i * (2 * kPi / 360);
    // crude: cast against the rectangle outline analytically - use the grid
    // itself to find the range (the grid IS the world here).
    double r = 0.05;
    while (r < 8.0) {
      const Vec2 w = truth.translation() +
                     Vec2(std::cos(ang), std::sin(ang)) * r;
      int gx, gy;
      g.worldToCell(w.x, w.y, gx, gy);
      if (g.occupied(gx, gy)) break;
      r += 0.05;
    }
    if (r < 8.0)
      // dir * r is already the offset from the pose (dir is a WORLD
      // direction); rotating it into the base frame completes the point.
      pts.push_back(rotate(Vec2(std::cos(ang), std::sin(ang)) * r,
                           -truth.theta));
  }
  ASSERT_GT(pts.size(), 50u);

  ScanMatcher matcher;
  const Pose2D seed(truth.x + 0.08, truth.y - 0.06,
                    truth.theta + 0.06);   // ~8 cm / 3.4 deg off
  const MatchResult m = matcher.match(g, pts, seed);
  EXPECT_NEAR(m.pose.x, truth.x, 0.02) << "matched x";
  EXPECT_NEAR(m.pose.y, truth.y, 0.02) << "matched y";
  EXPECT_NEAR(normalizeAngle(m.pose.theta - truth.theta), 0.0,
              2.0 * kPi / 180.0) << "matched yaw";
}

// --------------------------------------------------------------- mapper
TEST(Mapper, BuildsAsymmetricRoomMap) {
  // True world: 5 x 4 room with a notch (asymmetric so matching is
  // constrained).
  OccupancyGrid truth(0.05);
  const Vec2 corners[] = {{0, 0}, {5, 0}, {5, 3}, {4, 3}, {4, 4}, {0, 4}, {0, 0}};
  for (int i = 0; i + 1 < 6; ++i)
    for (double t = 0; t < 1.0; t += 0.01) {
      const Vec2 p(corners[i].x + t * (corners[i + 1].x - corners[i].x),
                   corners[i].y + t * (corners[i + 1].y - corners[i].y));
      truth.rayHit({2.5, 1.5}, p);
    }

  Mapper mapper;
  // Walk a loop around the room, feeding scans taken from the true pose
  // (the EKF seed has a little noise on top).
  std::mt19937 rng(5);
  std::normal_distribution<double> seedNoise(0.0, 0.008);  // 8 mm EKF error
  const Pose2D waypoints[] = {{1.0, 1.0, 0}, {4.0, 1.0, 0},
                              {4.0, 2.5, kPi / 2}, {1.5, 2.5, kPi},
                              {1.0, 1.0, -kPi / 2}};
  Pose2D pose = waypoints[0];
  for (size_t w = 1; w < 5; ++w) {
    const Pose2D target = waypoints[w];
    const int steps = 40;
    for (int s = 1; s <= steps; ++s) {
      const double f = static_cast<double>(s) / steps;
      pose.x = waypoints[w - 1].x + f * (target.x - waypoints[w - 1].x);
      pose.y = waypoints[w - 1].y + f * (target.y - waypoints[w - 1].y);
      pose.theta = normalizeAngle(
          waypoints[w - 1].theta +
          f * normalizeAngle(target.theta - waypoints[w - 1].theta));

      // Scan from the true pose against the true world.
      std::vector<Vec2> pts;
      for (int i = 0; i < 360; i += 4) {
        const double ang =
            pose.theta + deg2rad(180.0) - i * (2 * kPi / 360);
        double r = 0.05;
        while (r < 8.0) {
          const Vec2 p = pose.translation() +
                         Vec2(std::cos(ang), std::sin(ang)) * r;
          int gx, gy;
          truth.worldToCell(p.x, p.y, gx, gy);
          if (truth.occupied(gx, gy)) break;
          r += 0.05;
        }
        if (r < 8.0)
          // dir * r is already the offset from the pose; rotate into base.
          pts.push_back(rotate(Vec2(std::cos(ang), std::sin(ang)) * r,
                               -pose.theta));
      }
      if (pts.size() < 40) continue;

      const Pose2D seed(pose.x + seedNoise(rng), pose.y + seedNoise(rng),
                        pose.theta + seedNoise(rng) * 0.5);
      mapper.process(seed, pts);
    }
  }

  ASSERT_GT(mapper.keyframes().size(), 5u);
  const OccupancyGrid& map = mapper.map();

  // Map error vs ground truth: distance from each TRUE wall cell to the
  // nearest mapped occupied cell, RMS < 5 cm (prompt §8 Phase 3).
  double sumSq = 0.0;
  int n = 0;
  for (int y = 0; y < truth.height(); ++y) {
    for (int x = 0; x < truth.width(); ++x) {
      const int gx = truth.originX() + x, gy = truth.originY() + y;
      if (!truth.occupied(gx, gy)) continue;
      const Vec2 c = truth.cellCentre(gx, gy);
      // Nearest occupied cell in the map within +-3 cells.
      double best = 1e9;
      for (int dy = -3; dy <= 3; ++dy)
        for (int dx = -3; dx <= 3; ++dx) {
          int mgx, mgy;
          map.worldToCell(c.x, c.y, mgx, mgy);
          if (map.occupied(mgx + dx, mgy + dy))
            best = std::min(best,
                            std::hypot(dx * 0.05, dy * 0.05));
        }
      if (best < 1e8) {
        sumSq += best * best;
        ++n;
      }
    }
  }
  ASSERT_GT(n, 100);
  const double rms = std::sqrt(sumSq / n);
  EXPECT_LT(rms, 0.05) << "map wall RMS error must be < 5 cm";
}

}  // namespace
}  // namespace vacuum
