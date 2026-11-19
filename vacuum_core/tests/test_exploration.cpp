// test_exploration.cpp - Phase 6: frontier detection and goal choice.

#include <gtest/gtest.h>

#include "vacuum/frontier.h"
#include "vacuum/occupancy_grid.h"

namespace vacuum {
namespace {

// The test room with only the LEFT room known (free + walls); the right
// room is untouched = unknown. The divider gap cells are unknown too, so
// the left room's edge at the gap is a frontier.
OccupancyGrid partiallyKnownRoom() {
  OccupancyGrid g(0.05);   // everything starts unknown (logOdds 0)
  // Paint the walls of the whole floor plan (known structure).
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
  paint(2, 0, 2, 4);   // full divider for now; we free the gap below
  // Left room interior = free, filled FLUSH against the walls by INTEGER
  // cell indices: sweeping world coordinates with `x += 0.05` drifts in
  // floating point and skips cells, leaving phantom interior frontiers.
  for (int gy = 1; gy <= 79; ++gy)
    for (int gx = 1; gx <= 39; ++gx)
      g.setLogOdds(gx, gy, -1.0f);
  // Open the gap (y = 3..4 at x = 2) back to UNKNOWN - it is unexplored.
  for (int gy = 60; gy <= 79; ++gy)
    for (int gx = 38; gx <= 41; ++gx)
      g.setLogOdds(gx, gy, 0.0f);
  return g;
}

OccupancyGrid fullyKnownRoom() {
  OccupancyGrid g = partiallyKnownRoom();
  // Explore the rest: every still-unknown interior cell becomes free
  // (walls are occupied already and stay untouched). Integer cell sweep.
  for (int gy = 1; gy <= 79; ++gy)
    for (int gx = 1; gx <= 119; ++gx)
      if (g.logOdds(gx, gy) == 0.0f) g.setLogOdds(gx, gy, -1.0f);
  return g;
}

TEST(Frontier, FindsTheGapEdgeInPartialMap) {
  const OccupancyGrid g = partiallyKnownRoom();
  FrontierExplorer ex;
  const auto cs = ex.clusters(g);
  ASSERT_FALSE(cs.empty()) << "partial map must have frontiers";
  // The frontier hugs the unexplored side of the divider (x ~ 2): the
  // unknown strip left of the wall plus the open gap.
  bool nearDivider = false;
  for (const auto& c : cs)
    if (std::abs(c.centroid.x - 2.0) < 0.6) nearDivider = true;
  EXPECT_TRUE(nearDivider) << "frontier should hug the unexplored divider";
}

TEST(Frontier, BestGoalPointsAtTheFrontier) {
  const OccupancyGrid g = partiallyKnownRoom();
  FrontierExplorer ex;
  const Pose2D pose(1.0, 1.0, 0.0);   // robot in the known room
  const auto goal = ex.bestGoal(g, pose);
  ASSERT_TRUE(goal.has_value());
  // Goal is in the KNOWN room, standing off from the gap edge.
  EXPECT_LT(goal->x, 2.0) << "goal must stay in known free space";
  EXPECT_GT(goal->y, 2.0) << "goal should head toward the gap";
}

TEST(Frontier, ExplorationFinishesOnFullyKnownMap) {
  const OccupancyGrid g = fullyKnownRoom();
  FrontierExplorer ex;
  EXPECT_TRUE(ex.clusters(g).empty());
  EXPECT_FALSE(ex.bestGoal(g, Pose2D(1.0, 1.0, 0.0)).has_value());
}

TEST(Frontier, TinyFrontiersAreIgnored) {
  // A map whose only frontier is a 2-cell sliver (< 0.5 m extent).
  OccupancyGrid g(0.05);
  // One free cell next to one unknown cell, surrounded by walls.
  auto wall = [&](double x, double y) {
    int gx, gy;
    g.worldToCell(x, y, gx, gy);
    g.setLogOdds(gx, gy, OccupancyGrid::kLOddMax);
  };
  for (double t = 0; t <= 1.0; t += 0.02) {
    wall(0.5 + t * 0.4, 0.5); wall(0.5 + t * 0.4, 0.9);
    wall(0.5, 0.5 + t * 0.4); wall(0.9, 0.5 + t * 0.4);
  }
  int gx, gy;
  g.worldToCell(0.6, 0.7, gx, gy);
  g.setLogOdds(gx, gy, -1.0f);              // the only free cell
  g.worldToCell(0.7, 0.7, gx, gy);
  g.setLogOdds(gx, gy, 0.0f);               // neighbour unknown -> frontier

  FrontierExplorer ex;
  const auto cs = ex.clusters(g);
  ASSERT_EQ(cs.size(), 1u);
  EXPECT_LT(cs[0].extentM, 0.5);
  // The only frontier is too small: exploration is finished.
  EXPECT_FALSE(ex.bestGoal(g, Pose2D(0.6, 0.7, 0.0)).has_value());
}

}  // namespace
}  // namespace vacuum
