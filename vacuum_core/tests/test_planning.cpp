// test_planning.cpp - Phase 4 core: costmap layers + A* planner.

#include <gtest/gtest.h>

#include "vacuum/astar.h"
#include "vacuum/costmap.h"
#include "vacuum/occupancy_grid.h"

namespace vacuum {
namespace {

// Paint cells along a segment at full occupancy. Building test worlds by
// raycasting along the wall does NOT work: each ray clears (miss deltas)
// the cells it passes, including walls earlier rays painted - a sensor ray
// never continues past the surface it hits.
void paintLine(OccupancyGrid& g, Vec2 a, Vec2 b) {
  const double len = (b - a).norm();
  const int steps = std::max(2, static_cast<int>(len / (g.resolution() * 0.5)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const double x = a.x + t * (b.x - a.x);
    const double y = a.y + t * (b.y - a.y);
    int gx, gy;
    g.worldToCell(x, y, gx, gy);
    g.setLogOdds(gx, gy, OccupancyGrid::kLOddMax);
  }
}

OccupancyGrid roomWithWall() {
  // 6 x 4 m room with a wall down the middle, 1 m gap at the top.
  OccupancyGrid g(0.05);
  // A fresh grid is UNKNOWN everywhere; a mapped room has seen its floor.
  // Explicit bounds: `x < g.width()` would stop at the initial 64-cell
  // tile and never trigger the growth that widens the loop.
  const int W = static_cast<int>(std::ceil(6.5 / 0.05));
  const int H = static_cast<int>(std::ceil(4.5 / 0.05));
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x)
      g.setLogOdds(x, y, -1.0f);
  paintLine(g, {0, 0}, {6, 0});
  paintLine(g, {6, 0}, {6, 4});
  paintLine(g, {6, 4}, {0, 4});
  paintLine(g, {0, 4}, {0, 0});
  paintLine(g, {3, 0}, {3, 3});   // divider; gap y=3..4
  return g;
}

TEST(Costmap, StaticAndInflation) {
  RobotConfig cfg;
  Costmap cm(cfg);
  cm.setStatic(roomWithWall());

  // Far from anything: free.
  EXPECT_EQ(cm.costAt(1.0, 1.0), Costmap::kFree);
  // Inside a wall: lethal.
  EXPECT_EQ(cm.costAt(3.0, 1.5), Costmap::kLethal);
  // Just outside the wall (0.05..inflateRadius away): inflated cost.
  const double r = cm.inflateRadius();   // 0.23 + 0.10 = 0.33 m
  EXPECT_GT(cm.costAt(3.0 + r * 0.5, 1.5), 0);
  EXPECT_LT(cm.costAt(3.0 + r * 0.5, 1.5), Costmap::kMaxInflated + 1);
  // Beyond the radius: free again.
  EXPECT_EQ(cm.costAt(3.0 + r + 0.2, 1.5), Costmap::kFree);
}

TEST(Costmap, KeepoutIsAtLeastTwoCellsThick) {
  RobotConfig cfg;
  Costmap cm(cfg);
  cm.setStatic(roomWithWall());
  // No-go square in the left room.
  cm.addKeepout({{1.0, 1.0}, {2.0, 1.0}, {2.0, 2.0}, {1.0, 2.0}});

  EXPECT_EQ(cm.costAt(1.5, 1.5), Costmap::kLethal);
  // A 1-cell dilation runs around the polygon: the immediately adjacent
  // cells are lethal too (>= 2 cells thick everywhere). The next cell out
  // is merely inflated.
  EXPECT_EQ(cm.costAt(1.0 - 0.04, 1.5), Costmap::kLethal);
  EXPECT_EQ(cm.costAt(1.5, 1.0 - 0.04), Costmap::kLethal);
  EXPECT_GT(cm.costAt(1.0 - 0.10, 1.5), 0);
  EXPECT_LT(cm.costAt(1.0 - 0.10, 1.5), Costmap::kLethal);
  // Outside the inflated band: unaffected.
  EXPECT_EQ(cm.costAt(0.5, 0.5), Costmap::kFree);

  cm.clearKeepouts();
  EXPECT_EQ(cm.costAt(1.5, 1.5), Costmap::kFree);
}

TEST(Costmap, LiveObstaclesDecayAfterTwoSeconds) {
  RobotConfig cfg;
  Costmap cm(cfg);
  cm.setStatic(roomWithWall());

  cm.setObstacles({{1.0, 1.0}}, /*nowS=*/10.0);
  EXPECT_EQ(cm.costAt(1.0, 1.0), Costmap::kLethal);

  // Re-stamped by the next scan: still there.
  cm.setObstacles({{1.0, 1.0}}, 11.0);
  EXPECT_EQ(cm.costAt(1.0, 1.0), Costmap::kLethal);

  // No scan re-stamps it; the decay window (>2 s since the 11.0 stamp)
  // passes -> it fades.
  cm.expire(13.5);
  EXPECT_EQ(cm.costAt(1.0, 1.0), Costmap::kFree);
}

TEST(AStar, PlansAcrossOpenSpace) {
  RobotConfig cfg;
  Costmap cm(cfg);
  cm.setStatic(roomWithWall());

  AStarPlanner planner;
  const auto res = planner.plan(cm, {1.0, 1.0}, {1.0, 3.0});
  ASSERT_TRUE(res.ok);
  EXPECT_NEAR(res.path.front().x, 1.0, 1e-9);
  EXPECT_NEAR(res.path.front().y, 1.0, 1e-9);
  EXPECT_NEAR(res.path.back().x, 1.0, 1e-9);
  EXPECT_NEAR(res.path.back().y, 3.0, 1e-9);
  // Densified: consecutive points <= ~0.06 m apart (0.05 spacing + corner).
  for (size_t i = 1; i < res.path.size(); ++i) {
    const double d = (res.path[i] - res.path[i - 1]).norm();
    EXPECT_LT(d, 0.061) << "at " << i;
  }
}

TEST(AStar, GoesAroundTheWallThroughTheGap) {
  RobotConfig cfg;
  Costmap cm(cfg);
  cm.setStatic(roomWithWall());

  AStarPlanner planner;
  // Left room bottom -> right room bottom: must go up through the 1 m gap.
  const auto res = planner.plan(cm, {1.5, 0.5}, {4.5, 0.5});
  ASSERT_TRUE(res.ok);
  // No path point may sit in a lethal cell (the wall or its inflation
  // band is fine to approach, but never enter).
  for (const Vec2& p : res.path)
    EXPECT_NE(cm.costAt(p.x, p.y), Costmap::kLethal) << "at " << p.x << "," << p.y;
  // It must actually pass near the gap (y > 2.5 at the divider x=3).
  bool crossedHigh = false;
  for (const Vec2& p : res.path)
    if (std::abs(p.x - 3.0) < 0.3 && p.y > 2.5) crossedHigh = true;
  EXPECT_TRUE(crossedHigh) << "path should use the top gap";
}

TEST(AStar, RefusesGoalsInsideLethal) {
  RobotConfig cfg;
  Costmap cm(cfg);
  cm.setStatic(roomWithWall());

  AStarPlanner planner;
  const auto res = planner.plan(cm, {1.0, 1.0}, {3.0, 1.5});  // in the wall
  EXPECT_FALSE(res.ok);
}

TEST(AStar, UnknownCellsAreCostlyButCrossable) {
  // A grid that is mostly unknown (fresh map) with a known start/goal band.
  OccupancyGrid g(0.05);
  // Paint a known corridor by raytracing along y=1.
  for (double x = 0; x <= 4.0; x += 0.1) g.rayHit({0.0, 1.0}, {x, 1.0});
  // Plus walls so the corridor has shape.
  g.rayHit({2.0, 1.0}, {0.0, 0.5});
  g.rayHit({2.0, 1.0}, {4.0, 0.5});
  g.rayHit({2.0, 1.0}, {0.0, 1.5});
  g.rayHit({2.0, 1.0}, {4.0, 1.5});

  Costmap cm(RobotConfig{});
  cm.setStatic(g);
  AStarPlanner planner;
  const auto res = planner.plan(cm, {0.3, 1.0}, {3.7, 1.0});
  EXPECT_TRUE(res.ok);
}

}  // namespace
}  // namespace vacuum
