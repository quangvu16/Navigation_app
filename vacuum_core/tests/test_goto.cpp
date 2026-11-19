// test_goto.cpp - Phase 4 acceptance: RPP + GoToController driving the
// simulator through10 random goals (prompt §8: ≥9 reached within10 cm,
// robot never touches an obstacle).

#include <gtest/gtest.h>

#include <random>

#include "sim/sim_robot.h"
#include "vacuum/costmap.h"
#include "vacuum/goto_controller.h"
#include "vacuum/occupancy_grid.h"
#include "vacuum/rpp.h"

namespace vacuum {
namespace {

// Same asymmetric room as test_localization (divider at x=2: no mirror
// symmetry, so goals are unique).
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
  paint(2, 0, 2, 3);   // divider at x=2 (asymmetric!), gap y=3..4
  return g;
}

// Convert the occupancy grid into a SimWorld cell array (0 free/1 occ/2 unk).
SimWorld worldFrom(const OccupancyGrid& g) {
  std::vector<uint8_t> cells(static_cast<size_t>(g.width()) * g.height());
  for (int y = 0; y < g.height(); ++y)
    for (int x = 0; x < g.width(); ++x) {
      const int gx = g.originX() + x, gy = g.originY() + y;
      cells[static_cast<size_t>(y) * g.width() + x] =
          g.occupied(gx, gy) ? SimWorld::kOccupied
          : g.free(gx, gy)  ? SimWorld::kFree
                            : SimWorld::kUnknown;
    }
  return SimWorld(g.width(), g.height(), g.resolution(),
                  Vec2(g.originX() * g.resolution(),
                       g.originY() * g.resolution()),
                  cells.data());
}

TEST(Rpp, FollowsStraightPathAndSlowsOnSharpTurn) {
  RobotConfig cfg;
  Costmap cm(cfg);
  cm.setStatic(testRoom());
  RegulatedPurePursuit rpp;

  // Straight path east: forward command, no rotation.
  const std::vector<Vec2> straight = {{1.0, 1.0}, {1.5, 1.0}, {1.9, 1.0}};
  auto c = rpp.step(cm, Pose2D(1.0, 1.0, 0.0), straight);
  EXPECT_FALSE(c.goalReached);
  EXPECT_GT(c.v, 0.05);
  EXPECT_NEAR(c.w, 0.0, 0.35);
  EXPECT_FALSE(c.blocked);

  // Path behind the robot: rotate-in-place (heading error > 60 deg).
  const std::vector<Vec2> behind = {{1.0, 1.0}, {0.5, 1.0}};
  c = rpp.step(cm, Pose2D(1.0, 1.0, 0.0), behind);
  EXPECT_TRUE(c.rotateInPlace);
  EXPECT_NEAR(c.v, 0.0, 1e-9);
  EXPECT_NE(c.w, 0.0);
}

TEST(Rpp, RolloutBrakesBeforeWall) {
  RobotConfig cfg;
  Costmap cm(cfg);
  cm.setStatic(testRoom());
  RegulatedPurePursuit rpp;

  // Driving at the divider from0.4 m: the1 s rollout (0.35 m/s) would
  // enter the wall -> blocked, command zeroed.
  const std::vector<Vec2> path = {{1.6, 1.5}, {2.0, 1.5}};
  const auto c = rpp.step(cm, Pose2D(1.6, 1.5, 0.0), path);
  EXPECT_TRUE(c.blocked);
  EXPECT_NEAR(c.v, 0.0, 1e-9);
  EXPECT_NEAR(c.w, 0.0, 1e-9);
}

TEST(GoTo, RejectsGoalsInsideWalls) {
  RobotConfig cfg;
  Costmap cm(cfg);
  cm.setStatic(testRoom());
  GoToController gc;
  EXPECT_FALSE(gc.setGoal(cm, 2.0, 1.5));   // inside the divider
  EXPECT_EQ(gc.state(), GoToState::REJECTED);
  EXPECT_TRUE(gc.rejected());
}

TEST(GoTo, TenRandomGoalsAcceptance) {
  RobotConfig cfg;
  const OccupancyGrid room = testRoom();
  Costmap cm(cfg);
  cm.setStatic(room);
  const SimWorld world = worldFrom(room);

  std::mt19937 rng(1234);
  // Sample goals from free interior cells (keep clear of walls/divider).
  std::vector<Vec2> freePts;
  for (int y = 0; y < room.height(); ++y)
    for (int x = 0; x < room.width(); ++x) {
      const int gx = room.originX() + x, gy = room.originY() + y;
      if (!room.free(gx, gy)) continue;
      const Vec2 c = room.cellCentre(gx, gy);
      if (c.x < 0.3 || c.x > 5.7 || c.y < 0.3 || c.y > 3.7) continue;
      if (std::abs(c.x - 2.0) < 0.4) continue;   // divider buffer
      freePts.push_back(c);
    }
  ASSERT_GT(freePts.size(), 50u);

  int reached = 0;
  int touches = 0;
  const double mpsPerRpm = 2.0 * kPi * cfg.wheelRadiusM / 60.0;

  for (int trial = 0; trial < 10; ++trial) {
    const Vec2 goal = freePts[static_cast<size_t>(rng()) % freePts.size()];
    // Fresh noisy robot per trial (real dynamics: lag + slip + gyro noise).
    SimRobot robot(cfg, SimWorld(world), SimNoise(),
                   static_cast<uint32_t>(100 + trial));
    robot.setPose(Pose2D(1.0, 1.0, 0.0));

    GoToController gc;
    ASSERT_TRUE(gc.setGoal(cm, goal.x, goal.y)) << "goal " << trial;

    bool ok = false, touched = false;
    for (int step = 0; step < 4000; ++step) {   //80 s at50 Hz
      const Pose2D truth = robot.sensors().truePose;
      if (cm.costAt(truth.x, truth.y) == Costmap::kLethal) {
        touched = true;
        break;
      }
      const GoToCommand cmd = gc.step(cm, truth, step * 20L);
      if (cmd.state == GoToState::REACHED) { ok = true; break; }
      if (cmd.state == GoToState::STUCK) break;

      const double vl = (cmd.v - cmd.w * cfg.trackWidthM * 0.5) / mpsPerRpm;
      const double vr = (cmd.v + cmd.w * cfg.trackWidthM * 0.5) / mpsPerRpm;
      robot.setWheelRpm(vl, vr);
      robot.step(0.02);
    }
    if (touched) {
      ++touches;
      ADD_FAILURE() << "trial " << trial << " touched an obstacle at ("
                    << robot.sensors().truePose.x << ","
                    << robot.sensors().truePose.y << ")";
    } else if (ok) {
      ++reached;
      // Within10 cm of the goal at the moment of REACHED.
      const Pose2D t = robot.sensors().truePose;
      EXPECT_NEAR(std::hypot(t.x - goal.x, t.y - goal.y), 0.0, 0.10)
          << "trial " << trial;
    } else {
      ADD_FAILURE() << "trial " << trial << " timed out or got stuck (goal "
                    << goal.x << "," << goal.y << ")";
    }
  }

  EXPECT_EQ(touches, 0) << "robot must never touch an obstacle";
  EXPECT_GE(reached, 9) << "≥9 of10 goals must be reached (prompt §8)";
}

}  // namespace
}  // namespace vacuum
