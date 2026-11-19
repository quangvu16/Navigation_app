// astar.h - 8-connected A* on the costmap + path smoothing + densify to
// 0.05 m (prompt §5 global planner).
//
// Cost of a step = distance * (1 + cellCost/50), so inflation is respected
// without making the planner afraid of mild costs; lethal (254) cells are
// never entered; unknown (255) only when allow_unknown (Nav2 hint).

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "vacuum/costmap.h"
#include "vacuum/math.h"

namespace vacuum {

struct PlanResult {
  std::vector<Vec2> path;   // world metres, start -> goal
  bool ok = false;
};

class AStarPlanner {
 public:
  struct Params {
    bool allowUnknown = true;
    double unknownCost = 4.0;   // multiplier on unknown cells
    double densifyM = 0.05;     // prompt: densify to 0.05 m
    int smoothIters = 40;       // shortcut smoothing passes
  };

  // Two ctors (nested-Params NSDMI rule, as ScanMatcher/Ekf/Mapper).
  AStarPlanner() : p_() {}
  explicit AStarPlanner(const Params& p) : p_(p) {}

  PlanResult plan(const Costmap& map, Vec2 start, Vec2 goal) const {
    PlanResult out;
    if (map.width() <= 0) return out;

    const auto toCell = [&](Vec2 p) {
      return std::pair<int, int>(
          static_cast<int>(std::floor(p.x / map.resolution())),
          static_cast<int>(std::floor(p.y / map.resolution())));
    };
    const auto toWorld = [&](int gx, int gy) {
      return Vec2((gx + 0.5) * map.resolution(), (gy + 0.5) * map.resolution());
    };

    const auto [sx, sy] = toCell(start);
    const auto [gx, gy] = toCell(goal);
    if (!map.isTraversable(toWorld(sx, sy).x, toWorld(sx, sy).y,
                           p_.allowUnknown) &&
        map.cost(sx, gy) == Costmap::kLethal) {
      // start in lethal: refuse (prompt §7: clear message upstream)
      return out;
    }
    if (map.cost(gx, gy) == Costmap::kLethal) return out;  // goal in lethal

    const int w = map.width(), h = map.height();
    const auto id = [&](int x, int y) { return y * w + x; };
    const size_t n = static_cast<size_t>(w) * h;

    std::vector<float> gScore(n, std::numeric_limits<float>::infinity());
    std::vector<int32_t> came(n, -1);
    std::vector<uint8_t> closed(n, 0);

    // Binary heap of (f, index) - std::push_heap with greater comparator.
    std::vector<std::pair<float, int32_t>> heap;
    auto push = [&](float f, int32_t i) {
      heap.push_back({f, i});
      std::push_heap(heap.begin(), heap.end(),
                     std::greater<std::pair<float, int32_t>>());
    };
    auto pop = [&]() {
      std::pop_heap(heap.begin(), heap.end(),
                    std::greater<std::pair<float, int32_t>>());
      const auto top = heap.back();
      heap.pop_back();
      return top;
    };

    const int startId = id(sx, sy);
    const int goalId = id(gx, gy);
    gScore[startId] = 0.0f;
    push(heuristic(sx, sy, gx, gy), startId);

    static const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    bool found = false;
    while (!heap.empty()) {
      const auto [f, cur] = pop();
      if (closed[cur]) continue;
      closed[cur] = 1;
      if (cur == goalId) { found = true; break; }
      const int cx = cur % w, cy = cur / w;
      for (int k = 0; k < 8; ++k) {
        const int nx = cx + DX[k], ny = cy + DY[k];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const int ni = id(nx, ny);
        if (closed[ni]) continue;
        const uint8_t c = map.cost(nx, ny);
        if (c == Costmap::kLethal) continue;
        float mult = 1.0f;
        if (c == Costmap::kUnknown) {
          if (!p_.allowUnknown) continue;
          mult = static_cast<float>(p_.unknownCost);
        } else {
          mult = 1.0f + c / 50.0f;
        }
        const float step = (DX[k] != 0 && DY[k] != 0) ? 1.41421356f : 1.0f;
        const float ng = gScore[cur] + step * mult;
        if (ng < gScore[ni]) {
          gScore[ni] = ng;
          came[ni] = cur;
          push(ng + heuristic(nx, ny, gx, gy), ni);
        }
      }
    }
    if (!found) return out;

    // Reconstruct cell path, convert to world, then smooth + densify.
    std::vector<Vec2> cells;
    for (int cur = goalId; cur != -1; cur = came[cur])
      cells.push_back(toWorld(cur % w, cur / w));
    std::reverse(cells.begin(), cells.end());
    cells.front() = start;      // exact endpoints
    cells.back() = goal;

    out.path = smooth(map, cells);
    out.path = densify(out.path, p_.densifyM);
    out.ok = !out.path.empty();
    return out;
  }

 private:
  Params p_;

  static float heuristic(int x0, int y0, int x1, int y1) {
    const float dx = static_cast<float>(std::abs(x0 - x1));
    const float dy = static_cast<float>(std::abs(y0 - y1));
    return std::max(dx, dy) + 0.41421356f * std::min(dx, dy);  // octile
  }

  // String-pulling: drop waypoints whose straight segment stays clear.
  std::vector<Vec2> smooth(const Costmap& map,
                           const std::vector<Vec2>& path) const {
    if (path.size() < 3) return path;
    std::vector<Vec2> out;
    out.push_back(path.front());
    size_t i = 0;
    while (i + 1 < path.size()) {
      size_t j = path.size() - 1;
      for (; j > i + 1; --j)
        if (segmentFree(map, path[i], path[j])) break;
      out.push_back(path[j]);
      i = j;
    }
    return out;
  }

  bool segmentFree(const Costmap& map, Vec2 a, Vec2 b) const {
    const double len = (b - a).norm();
    const int steps = std::max(2, static_cast<int>(len / (map.resolution() * 0.5)));
    for (int s = 0; s <= steps; ++s) {
      const double t = static_cast<double>(s) / steps;
      const double x = a.x + t * (b.x - a.x);
      const double y = a.y + t * (b.y - a.y);
      if (!map.isTraversable(x, y, p_.allowUnknown)) return false;
    }
    return true;
  }

  static std::vector<Vec2> densify(const std::vector<Vec2>& path,
                                   double spacing) {
    if (path.size() < 2 || spacing <= 0) return path;
    std::vector<Vec2> out;
    out.push_back(path.front());
    for (size_t i = 1; i < path.size(); ++i) {
      const Vec2 a = out.back();
      const Vec2 b = path[i];
      const double len = (b - a).norm();
      const int n = static_cast<int>(std::floor(len / spacing));
      for (int k = 1; k <= n; ++k) {
        const double t = static_cast<double>(k) * spacing / len;
        if (t >= 1.0) break;
        out.push_back(a + (b - a) * t);
      }
      out.push_back(b);
    }
    return out;
  }
};

}  // namespace vacuum
