// test_rooms_coverage.cpp - Phase 7: room segmentation + boustrophedon
// coverage (prompt §8:3-room sim map ->3 rooms; coverage >=90% per room).

#include <gtest/gtest.h>

#include <cmath>

#include "vacuum/coverage.h"
#include "vacuum/occupancy_grid.h"
#include "vacuum/segmenter.h"

namespace vacuum {
namespace {

// Three rooms side by side (6 x 4 m), dividers at x=2 and x=4 with1 m
// doorways (top of divider1, bottom of divider2). INTEGER cell painting
// only - world-coordinate sweeps drift and skip cells (test_exploration).
OccupancyGrid threeRoomMap() {
  OccupancyGrid g(0.05);
  const int W = 120, H = 80;   // 6 x 4 m at0.05
  // Interior free.
  for (int gy = 1; gy <= H - 2; ++gy)
    for (int gx = 1; gx <= W - 2; ++gx) g.setLogOdds(gx, gy, -1.0f);
  // Outer walls.
  auto paintLine = [&](int gx0, int gy0, int gx1, int gy1) {
    const int n = std::max(std::abs(gx1 - gx0), std::abs(gy1 - gy0));
    for (int i = 0; i <= n; ++i) {
      const int gx = gx0 + (gx1 - gx0) * i / n;
      const int gy = gy0 + (gy1 - gy0) * i / n;
      g.setLogOdds(gx, gy, OccupancyGrid::kLOddMax);
    }
  };
  paintLine(0, 0, W - 1, 0);
  paintLine(W - 1, 0, W - 1, H - 1);
  paintLine(W - 1, H - 1, 0, H - 1);
  paintLine(0, H - 1, 0, 0);
  // Dividers (cells40 and80), full height...
  paintLine(40, 0, 40, H - 1);
  paintLine(80, 0, 80, H - 1);
  // ...then open the doorways: divider1 door rows60..79 (y3..4 top),
  // divider2 door rows1..19 (y0.05..1 bottom).
  for (int gy = 60; gy <= 79; ++gy) g.setLogOdds(40, gy, -1.0f);
  for (int gy = 1; gy <= 19; ++gy) g.setLogOdds(80, gy, -1.0f);
  return g;
}

TEST(RoomSegmenter, ThreeRoomsFromThreeRoomMap) {
  const OccupancyGrid g = threeRoomMap();
  RoomSegmenter seg;
  std::vector<int16_t> labels;
  const auto rooms = seg.segment(g, labels);

  EXPECT_EQ(rooms.size(), 3u) << "3-room map must segment into3 rooms";
  // Three sample points, one per room, must carry three DIFFERENT ids.
  const int a = seg.labelAt(labels, g, 1.0, 2.0);
  const int b = seg.labelAt(labels, g, 3.0, 2.0);
  const int c = seg.labelAt(labels, g, 5.0, 2.0);
  EXPECT_GT(a, 0);
  EXPECT_GT(b, 0);
  EXPECT_GT(c, 0);
  EXPECT_NE(a, b);
  EXPECT_NE(b, c);
  EXPECT_NE(a, c);
  // Each room holds a meaningful area (>= a few m^2).
  for (const auto& r : rooms) EXPECT_GT(r.areaM2, 2.0);
}

TEST(RoomSegmenter, TinyClosetIsMergedNotARoom) {
  // Room A with a small closet (0.4 x 0.4 m inside a ring wall with a
  // door), room B on the other side of a divider. The closet's region is
  // far below the1 m^2 merge threshold (or its core dies during deeper
  // erosion) - either way it must NOT appear as its own room.
  OccupancyGrid g(0.05);
  for (int gy = 1; gy <= 78; ++gy)
    for (int gx = 1; gx <= 118; ++gx) g.setLogOdds(gx, gy, -1.0f);
  auto wall = [&](int gx, int gy) {
    g.setLogOdds(gx, gy, OccupancyGrid::kLOddMax);
  };
  // Outer box + middle divider at cell60 (x=3) with a door rows40..59.
  for (int gx = 0; gx <= 119; ++gx) { wall(gx, 0); wall(gx, 79); }
  for (int gy = 0; gy <= 79; ++gy) { wall(0, gy); wall(119, gy); }
  for (int gy = 0; gy <= 79; ++gy) wall(60, gy);
  for (int gy = 40; gy <= 59; ++gy) g.setLogOdds(60, gy, -1.0f);
  // Closet ring inside the left room: cells10..18 x10..18, door at
  // (18,14) (the right side of the ring).
  for (int gx = 10; gx <= 18; ++gx) { wall(gx, 10); wall(gx, 18); }
  for (int gy = 10; gy <= 18; ++gy) { wall(10, gy); wall(18, gy); }
  g.setLogOdds(18, 14, -1.0f);   // the closet doorway

  RoomSegmenter seg;
  std::vector<int16_t> labels;
  const auto rooms = seg.segment(g, labels);
  EXPECT_EQ(rooms.size(), 2u) << "closet must merge into its room";
  // The closet interior belongs to the same room as the room around it.
  const int roomA = seg.labelAt(labels, g, 1.4, 1.4);
  const int closet = seg.labelAt(labels, g, 0.7, 0.7);
  EXPECT_GT(roomA, 0);
  EXPECT_EQ(closet, roomA);
}

TEST(Coverage, BoustrophedonCoversRoomAbove90Percent) {
  const OccupancyGrid g = threeRoomMap();
  RoomSegmenter seg;
  std::vector<int16_t> labels;
  const auto rooms = seg.segment(g, labels);
  ASSERT_EQ(rooms.size(), 3u);

  // Take the middle room (sample point x=3).
  const int roomId = seg.labelAt(labels, g, 3.0, 2.0);
  ASSERT_GT(roomId, 0);

  CoverageParams cp;
  CoveragePlanner planner(cp);
  const auto path = planner.plan(g, labels, roomId);
  ASSERT_GT(path.size(), 10u) << "room must yield a real coverage path";

  // Simulate driving the path and painting with the brush footprint.
  CoverageTracker tracker;
  tracker.reset(g);
  tracker.setRoom(labels, roomId, cp.robotRadiusM);
  size_t i = 0;
  double x = path[0].x, y = path[0].y;
  while (i < path.size()) {
    const double dx = path[i].x - x, dy = path[i].y - y;
    const double d = std::hypot(dx, dy);
    if (d < 0.02) { ++i; continue; }
    const double step = std::min(0.02, d);
    x += dx / d * step;
    y += dy / d * step;
    tracker.markPose(x, y, cp.brushWidthM * 0.5);
  }

  EXPECT_GE(tracker.percent(), 90.0)
      << "coverage must reach >=90% per room (prompt §8)";
}

TEST(Coverage, EmptyRoomYieldsNoPath) {
  OccupancyGrid g(0.05);   // all unknown - no room
  RoomSegmenter seg;
  std::vector<int16_t> labels;
  seg.segment(g, labels);
  CoveragePlanner planner;
  EXPECT_TRUE(planner.plan(g, labels, 1).empty());
}

}  // namespace
}  // namespace vacuum
