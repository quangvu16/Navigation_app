// frontier.h - frontier-based exploration (prompt §5):
//   frontier clusters = free cells adjacent to unknown cells; score by
//   size / distance; go to the best; finish when no reachable frontier
//   cluster is larger than 0.5 m; return to start/dock afterwards.
//
// Cluster = connected component (4-neighbourhood) of frontier cells.
// Goal = the cluster centroid snapped to a free cell; the GoToController
// does the actual driving and its keepout/obstacle layers apply as usual.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "vacuum/math.h"
#include "vacuum/occupancy_grid.h"

namespace vacuum {

struct FrontierCluster {
  Vec2 centroid;      // world metres (mean of member cells)
  double extentM;     // bbox diagonal - the "larger than 0.5 m" size test
  int cells = 0;
};

class FrontierExplorer {
 public:
  struct Params {
    double minExtentM = 0.5;      // prompt: finish when none > 0.5 m
    double maxGoalDistM = 8.0;    // ignore frontiers farther than this
    double goalClearanceM = 0.15; // stand-off from the frontier itself
  };

  FrontierExplorer() : p_() {}
  explicit FrontierExplorer(const Params& p) : p_(p) {}

  // All frontier clusters in the map (largest first).
  std::vector<FrontierCluster> clusters(const OccupancyGrid& map) const {
    const int w = map.width(), h = map.height();
    std::vector<uint8_t> isFrontier(static_cast<size_t>(w) * h, 0);
    auto idx = [&](int x, int y) { return static_cast<size_t>(y) * w + x; };

    // Frontier cells: free, with at least one unknown 4-neighbour.
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        const int gx = map.originX() + x, gy = map.originY() + y;
        if (!map.free(gx, gy)) continue;
        bool unknown = false;
        static const int DX[4] = {1, -1, 0, 0};
        static const int DY[4] = {0, 0, 1, -1};
        for (int k = 0; k < 4 && !unknown; ++k) {
          const int nx = x + DX[k], ny = y + DY[k];
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
          if (!map.free(map.originX() + nx, map.originY() + ny) &&
              !map.occupied(map.originX() + nx, map.originY() + ny))
            unknown = true;   // unknown (neither free nor occupied)
        }
        if (unknown) isFrontier[idx(x, y)] = 1;
      }

    // Connected components (iterative flood fill).
    std::vector<uint8_t> seen(isFrontier.size(), 0);
    std::vector<FrontierCluster> out;
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        if (!isFrontier[idx(x, y)] || seen[idx(x, y)]) continue;
        // BFS this component.
        std::vector<std::pair<int, int>> stack = {{x, y}};
        seen[idx(x, y)] = 1;
        double sx = 0, sy = 0;
        int minX = x, maxX = x, minY = y, maxY = y, n = 0;
        while (!stack.empty()) {
          const auto [cx, cy] = stack.back();
          stack.pop_back();
          sx += cx; sy += cy;
          ++n;
          minX = std::min(minX, cx); maxX = std::max(maxX, cx);
          minY = std::min(minY, cy); maxY = std::max(maxY, cy);
          static const int DX[4] = {1, -1, 0, 0};
          static const int DY[4] = {0, 0, 1, -1};
          for (int k = 0; k < 4; ++k) {
            const int nx = cx + DX[k], ny = cy + DY[k];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            if (isFrontier[idx(nx, ny)] && !seen[idx(nx, ny)]) {
              seen[idx(nx, ny)] = 1;
              stack.push_back({nx, ny});
            }
          }
        }
        FrontierCluster c;
        const double res = map.resolution();
        c.centroid = map.cellCentre(map.originX() + static_cast<int>(sx / n),
                                    map.originY() + static_cast<int>(sy / n));
        c.extentM = std::hypot((maxX - minX + 1) * res,
                               (maxY - minY + 1) * res);
        c.cells = n;
        out.push_back(c);
      }
    std::sort(out.begin(), out.end(),
              [](const FrontierCluster& a, const FrontierCluster& b) {
                return a.cells > b.cells;
              });
    return out;
  }

  // Best exploration goal from `pose`: largest cluster within range,
  // scored size/(1+distance) (prompt). nullopt = exploration finished.
  std::optional<Vec2> bestGoal(const OccupancyGrid& map,
                               const Pose2D& pose) const {
    const auto cs = clusters(map);
    double bestScore = -1.0;
    std::optional<Vec2> best;
    for (const auto& c : cs) {
      if (c.extentM < p_.minExtentM) continue;   // too small: ignore
      const double d = std::hypot(c.centroid.x - pose.x,
                                  c.centroid.y - pose.y);
      if (d > p_.maxGoalDistM) continue;
      const double score = c.cells / (1.0 + d);
      if (score > bestScore) {
        bestScore = score;
        // Stand off from the frontier by goalClearanceM along the
        // pose->centroid direction (do not drive INTO the unknown).
        Vec2 dir(c.centroid.x - pose.x, c.centroid.y - pose.y);
        const double len = dir.norm();
        if (len > 1e-6) {
          dir = dir * (1.0 / len);
          best = Vec2(c.centroid.x - dir.x * p_.goalClearanceM,
                      c.centroid.y - dir.y * p_.goalClearanceM);
        } else {
          best = c.centroid;
        }
      }
    }
    return best;
  }

 private:
  Params p_;
};

}  // namespace vacuum
