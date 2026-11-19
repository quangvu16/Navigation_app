// goto_controller.h - Go-to-goal state machine (prompt §5):
//   PLANNING -> FOLLOWING (RPP) -> REACHED
//   blocked (RPP rollout) -> RECOVERING: back up0.15 m -> rotate90 deg ->
//   replan; after3 failed recoveries -> STUCK.
//   Goals inside lethal cells are REJECTED before planning (prompt §7).

#pragma once

#include <cmath>
#include <vector>

#include "vacuum/astar.h"
#include "vacuum/costmap.h"
#include "vacuum/math.h"
#include "vacuum/rpp.h"

namespace vacuum {

enum class GoToState {
  IDLE,
  REJECTED,     // goal in lethal / no plan possible
  PLANNING,
  FOLLOWING,
  RECOVERING,
  REACHED,
  STUCK,        // gave up after maxRecoveries
};

struct GoToCommand {
  double v = 0.0;
  double w = 0.0;
  GoToState state = GoToState::IDLE;
  bool rejected = false;   // set once when a goal is refused (UI message)
  bool stuck = false;      // set once when giving up
};

class GoToController {
 public:
  struct Params {
    RppParams rpp;
    double recoverBackupM = 0.15;     // prompt §5
    double recoverRotateRad = kPi / 2.0;
    double recoverBackupV = -0.10;
    double recoverRotateW = 0.6;
    int maxRecoveries = 3;            // prompt: give up after3 tries
    int planEveryMs = 1000;           // replan at1 Hz or when blocked
  };

  GoToController() : p_() {}
  explicit GoToController(const Params& p) : p_(p), rpp_(p.rpp) {}

  // Returns false (and latches REJECTED) when the goal is inside a lethal
  // cell - the UI shows a clear message (prompt §7).
  bool setGoal(const Costmap& map, double gx, double gy) {
    goal_ = Vec2(gx, gy);
    state_ = GoToState::PLANNING;
    rejected_ = false;
    stuck_ = false;
    recoverPhase_ = 0;
    recoverProgress_ = 0.0;
    tries_ = 0;
    lastPlanMs_ = -1'000'000;
    path_.clear();
    if (map.costAt(gx, gy) == Costmap::kLethal) {
      state_ = GoToState::REJECTED;
      rejected_ = true;
      return false;
    }
    return true;
  }

  void cancel() {
    state_ = GoToState::IDLE;
    path_.clear();
  }

  GoToState state() const { return state_; }
  bool rejected() const { return rejected_; }
  bool stuck() const { return stuck_; }
  const std::vector<Vec2>& path() const { return path_; }

  // One control tick (call at50 Hz). nowMs drives the1 Hz replan.
  GoToCommand step(const Costmap& map, const Pose2D& pose, long nowMs) {
    GoToCommand out;
    out.state = state_;
    if (state_ == GoToState::IDLE || state_ == GoToState::REJECTED ||
        state_ == GoToState::REACHED || state_ == GoToState::STUCK) {
      out.rejected = rejected_;
      out.stuck = stuck_;
      return out;
    }

    switch (state_) {
      case GoToState::PLANNING: {
        const auto plan = planner_.plan(map, pose.translation(), goal_);
        if (!plan.ok) {
          // No path: reject if the goal itself is unreachable-looking,
          // otherwise recover-and-retry (the world may open up).
          state_ = GoToState::RECOVERING;
          recoverPhase_ = 0;
          recoverProgress_ = 0.0;
          break;
        }
        path_ = plan.path;
        lastPlanMs_ = nowMs;
        state_ = GoToState::FOLLOWING;
        break;
      }

      case GoToState::FOLLOWING: {
        const RppCommand c = rpp_.step(map, pose, path_);
        if (c.goalReached) {
          state_ = GoToState::REACHED;
          out.v = 0.0;
          out.w = 0.0;
          out.state = state_;
          return out;
        }
        if (c.blocked) {
          state_ = GoToState::RECOVERING;
          recoverPhase_ = 0;   //0 = back up,1 = rotate,2 = replan
          recoverProgress_ = 0.0;
          break;
        }
        // Replan at1 Hz (prompt) - the map may have changed.
        if (nowMs - lastPlanMs_ >= p_.planEveryMs) {
          const auto plan = planner_.plan(map, pose.translation(), goal_);
          if (plan.ok) {
            path_ = plan.path;
            lastPlanMs_ = nowMs;
          }
        }
        out.v = c.v;
        out.w = c.w;
        out.state = state_;
        return out;
      }

      case GoToState::RECOVERING: {
        // Phase0: back up recoverBackupM; phase1: rotate recoverRotateRad;
        // phase2: replan (falls into PLANNING via the switch below).
        if (recoverPhase_ == 0) {
          recoverProgress_ += std::abs(p_.recoverBackupV) * 0.02;  // ~dt
          out.v = p_.recoverBackupV;
          out.w = 0.0;
          if (recoverProgress_ >= p_.recoverBackupM) {
            recoverPhase_ = 1;
            recoverProgress_ = 0.0;
          }
        } else if (recoverPhase_ == 1) {
          recoverProgress_ += std::abs(p_.recoverRotateW) * 0.02;
          out.v = 0.0;
          out.w = p_.recoverRotateW;
          if (recoverProgress_ >= p_.recoverRotateRad) {
            recoverPhase_ = 2;
            recoverProgress_ = 0.0;
            ++tries_;
            if (tries_ > p_.maxRecoveries) {
              state_ = GoToState::STUCK;
              stuck_ = true;
              out.v = 0.0;
              out.w = 0.0;
              out.state = state_;
              out.stuck = true;
              return out;
            }
          }
        } else {
          state_ = GoToState::PLANNING;   // replan after recovery
        }
        out.state = state_;
        return out;
      }

      default:
        break;
    }
    out.state = state_;
    return out;
  }

 private:
  Params p_;
  RegulatedPurePursuit rpp_;
  AStarPlanner planner_;
  GoToState state_ = GoToState::IDLE;
  std::vector<Vec2> path_;
  Vec2 goal_;
  bool rejected_ = false;
  bool stuck_ = false;
  int recoverPhase_ = 0;
  double recoverProgress_ = 0.0;
  int tries_ = 0;
  long lastPlanMs_ = -1'000'000;
};

}  // namespace vacuum
