// test_sim_world.cpp — grid DDA raycasting and PGM loading.

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>

#include "sim/sim_world.h"

namespace vacuum {
namespace {

TEST(SimWorld, RaycastHitsRoomWalls) {
  // 4 m room, 0.1 m walls -> inner wall surface 1.9 m from the centre.
  const SimWorld w = SimWorld::emptyRoom(4.0, 0.05, 0.10);
  const Vec2 centre(2.0, 2.0);
  EXPECT_NEAR(w.raycast(centre, 0.0, 8.0), 1.9, 0.02);           // +x
  EXPECT_NEAR(w.raycast(centre, kPi / 2, 8.0), 1.9, 0.02);       // +y
  EXPECT_NEAR(w.raycast(centre, kPi, 8.0), 1.9, 0.02);           // -x
  EXPECT_NEAR(w.raycast(centre, -kPi / 2, 8.0), 1.9, 0.02);      // -y
  // Corner direction: sqrt(2) * 1.9 along the diagonal.
  EXPECT_NEAR(w.raycast(centre, kPi / 4, 8.0), 1.9 * std::sqrt(2.0), 0.03);
}

TEST(SimWorld, RaycastRespectsMaxRange) {
  const SimWorld w = SimWorld::emptyRoom(4.0);
  EXPECT_NEAR(w.raycast(Vec2(2.0, 2.0), 0.0, 1.0), 1.0, 1e-9);
}

TEST(SimWorld, DdaDoesNotSkipThinWalls) {
  // 1-cell wall at column 10 — a stepping raycaster with a coarse step can
  // jump over this; the DDA must not.
  const int n = 20;
  std::vector<uint8_t> cells(n * n, SimWorld::kFree);
  for (int y = 0; y < n; ++y) cells[y * n + 10] = SimWorld::kOccupied;
  const SimWorld w(n, n, 0.1, Vec2(0, 0), cells.data());
  const double hit = w.raycast(Vec2(0.25, 0.55), 0.0, 5.0);
  EXPECT_NEAR(hit, 0.75, 0.02);  // wall face at x=1.0, ray starts at x=0.25
}

TEST(SimWorld, UnknownBlocksLikeOccupied) {
  const int n = 10;
  std::vector<uint8_t> cells(n * n, SimWorld::kFree);
  cells[5 * n + 5] = SimWorld::kUnknown;
  const SimWorld w(n, n, 0.1, Vec2(0, 0), cells.data());
  EXPECT_LT(w.raycast(Vec2(0.05, 0.55), 0.0, 5.0), 1.0);
}

TEST(SimWorld, PgmLoadFlipsRowsSoRow0IsBottom) {
  // 3x2 PGM: top row white, bottom row black (walls).
  const char* path = "/tmp/vacuum_test_sim_world.pgm";
  {
    std::ofstream out(path, std::ios::binary);
    out << "P5\n3 2\n255\n";
    const unsigned char top[3] = {255, 255, 255};
    const unsigned char bottom[3] = {0, 0, 0};
    out.write(reinterpret_cast<const char*>(top), 3);
    out.write(reinterpret_cast<const char*>(bottom), 3);
  }
  const SimWorld w = SimWorld::loadPgm(path, 0.1, Vec2(0, 0));
  std::remove(path);
  ASSERT_EQ(w.width(), 3);
  ASSERT_EQ(w.height(), 2);
  // Row 0 = bottom = the black row = occupied.
  EXPECT_EQ(w.cell(0, 0), SimWorld::kOccupied);
  EXPECT_EQ(w.cell(2, 0), SimWorld::kOccupied);
  // Row 1 = top = white = free.
  EXPECT_EQ(w.cell(1, 1), SimWorld::kFree);
}

TEST(SimWorld, OutOfGridRayEndsAtBorder) {
  const SimWorld w = SimWorld::emptyRoom(4.0);
  // From inside heading out through a gap is impossible (walls), but from the
  // centre with a tiny max range the result is the max range.
  EXPECT_NEAR(w.raycast(Vec2(2.0, 2.0), 1.234, 0.4), 0.4, 1e-9);
}

}  // namespace
}  // namespace vacuum
