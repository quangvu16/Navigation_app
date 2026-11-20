// segmenter.h - morphological room segmentation (prompt §5):
//   erode free space step by step until regions split (doorways pinch
//   first), label the split components as room cores, grow back by
//   wavefront so every free cell joins its nearest core, merge regions
//   smaller than 1 m^2 into their neighbour.
//
// Stable integer room IDs: a label grid (int16 per cell) + Room records.
// User renames/merges/splits are stored outside (app side) and reapplied
// on top of these stable IDs (prompt §5).

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <vector>

#include "vacuum/math.h"
#include "vacuum/occupancy_grid.h"

namespace vacuum {

struct Room {
  int id = -1;
  int cells = 0;          // free cells assigned
  double areaM2 = 0.0;    // cells * res^2
  Vec2 centroid;          // world metres
};

class RoomSegmenter {
 public:
  struct Params {
    double maxErodeM = 0.50;   // stop eroding beyond this (doorways pinch
                               // well before; rooms must not vanish)
    double minAreaM2 = 1.0;    // prompt: merge regions < 1 m^2
    int minCoreCells = 4;      // ignore specks during labelling
  };

  RoomSegmenter() : p_() {}
  explicit RoomSegmenter(const Params& p) : p_(p) {}

  // Segment `grid`; fills label grid (0 = not a room / wall / unknown,
  // otherwise room id >= 1). Returns the rooms, ids stable per call.
  std::vector<Room> segment(const OccupancyGrid& grid,
                            std::vector<int16_t>& labels) const {
    const int w = grid.width(), h = grid.height();
    const double res = grid.resolution();
    labels.assign(static_cast<size_t>(w) * h, 0);
    auto idx = [&](int x, int y) { return static_cast<size_t>(y) * w + x; };

    // Free mask.
    std::vector<uint8_t> free(static_cast<size_t>(w) * h, 0);
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x)
        free[idx(x, y)] = grid.free(grid.originX() + x,
                                    grid.originY() + y) ? 1 : 0;

    // --- erode layer by layer until the free region splits ----------------
    // Doorways pinch before rooms do; DIFFERENT doorways pinch at
    // different radii, so keep eroding (one layer per step) and remember
    // the mask with the MOST components seen within maxErodeM. If nothing
    // ever splits (one open room), best stays the full free space and the
    // whole area becomes one room.
    const int maxR = static_cast<int>(p_.maxErodeM / res);
    std::vector<uint8_t> mask = free;
    std::vector<uint8_t> best = free;
    int bestCount = countComponents(free, w, h);
    for (int r = 1; r <= maxR; ++r) {
      std::vector<uint8_t> next = erode(mask, w, h, 1);   // one layer
      if (isAllZero(next)) break;       // everything eroded: stop
      const int c = countComponents(next, w, h);
      // >= so ties prefer the DEEPER mask: an early split (tiny closet
      // pinching its door at r=1) must not shadow a later, more meaningful
      // split (the real rooms at r=10).
      if (c >= bestCount) {
        best = next;
        bestCount = c;
      }
      mask = next;
    }

    // --- label cores (skip specks) ---------------------------------------
    std::vector<int32_t> coreLabel(static_cast<size_t>(w) * h, 0);
    int nextId = 1;
    std::vector<std::pair<int, int>> coreCells;
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        if (!best[idx(x, y)] || coreLabel[idx(x, y)]) continue;
        // BFS component.
        std::vector<std::pair<int, int>> comp;
        std::queue<std::pair<int, int>> q;
        q.push({x, y});
        coreLabel[idx(x, y)] = nextId;
        while (!q.empty()) {
          const auto [cx, cy] = q.front();
          q.pop();
          comp.push_back({cx, cy});
          static const int DX[4] = {1, -1, 0, 0};
          static const int DY[4] = {0, 0, 1, -1};
          for (int k = 0; k < 4; ++k) {
            const int nx = cx + DX[k], ny = cy + DY[k];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            if (best[idx(nx, ny)] && !coreLabel[idx(nx, ny)]) {
              coreLabel[idx(nx, ny)] = nextId;
              q.push({nx, ny});
            }
          }
        }
        if (static_cast<int>(comp.size()) >= p_.minCoreCells) {
          for (const auto& c : comp) coreCells.push_back(c);
          ++nextId;
        } else {
          for (const auto& c : comp) coreLabel[idx(c.first, c.second)] = 0;
        }
      }

    // --- grow back: multi-source wavefront over free cells ----------------
    // Every free cell joins the nearest core's label (BFS layer order).
    std::queue<std::pair<int, int>> frontier;
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        const int16_t l = static_cast<int16_t>(coreLabel[idx(x, y)]);
        if (l) {
          labels[idx(x, y)] = l;
          frontier.push({x, y});
        }
      }
    static const int DX[4] = {1, -1, 0, 0};
    static const int DY[4] = {0, 0, 1, -1};
    while (!frontier.empty()) {
      const auto [cx, cy] = frontier.front();
      frontier.pop();
      const int16_t l = labels[idx(cx, cy)];
      for (int k = 0; k < 4; ++k) {
        const int nx = cx + DX[k], ny = cy + DY[k];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (free[idx(nx, ny)] && !labels[idx(nx, ny)]) {
          labels[idx(nx, ny)] = l;
          frontier.push({nx, ny});
        }
      }
    }

    // --- merge regions < minAreaM2 into the neighbour with the longest
    // shared border (prompt: merge regions < 1 m^2) ------------------------
    mergeSmallRegions(labels, free, w, h, res);

    // --- collect rooms ----------------------------------------------------
    std::vector<Room> rooms;
    std::vector<int> cellCount(static_cast<size_t>(nextId), 0);
    std::vector<double> sx(static_cast<size_t>(nextId), 0.0),
        sy(static_cast<size_t>(nextId), 0.0);
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        const int16_t l = labels[idx(x, y)];
        if (l <= 0) continue;
        cellCount[l]++;
        sx[l] += x;
        sy[l] += y;
      }
    for (int id = 1; id < nextId; ++id) {
      if (cellCount[id] == 0) continue;
      Room r;
      r.id = id;
      r.cells = cellCount[id];
      r.areaM2 = r.cells * res * res;
      const int cx = static_cast<int>(sx[id] / cellCount[id]);
      const int cy = static_cast<int>(sy[id] / cellCount[id]);
      r.centroid = grid.cellCentre(grid.originX() + cx,
                                   grid.originY() + cy);
      rooms.push_back(r);
    }
    return rooms;
  }

  // Room id at a world point (0 = none).
  int16_t labelAt(const std::vector<int16_t>& labels, const OccupancyGrid& g,
                  double wx, double wy) const {
    int gx, gy;
    g.worldToCell(wx, wy, gx, gy);
    const int x = gx - g.originX(), y = gy - g.originY();
    if (x < 0 || y < 0 || x >= g.width() || y >= g.height()) return 0;
    return labels[static_cast<size_t>(y) * g.width() + x];
  }

 private:
  Params p_;

  static bool isAllZero(const std::vector<uint8_t>& v) {
    for (uint8_t b : v)
      if (b) return false;
    return true;
  }

  // 4-connected component count over a w*h mask.
  static int countComponents(const std::vector<uint8_t>& m, int w, int h) {
    std::vector<uint8_t> seen(m.size(), 0);
    int count = 0;
    static const int DX[4] = {1, -1, 0, 0};
    static const int DY[4] = {0, 0, 1, -1};
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        const size_t i = static_cast<size_t>(y) * w + x;
        if (!m[i] || seen[i]) continue;
        ++count;
        std::vector<std::pair<int, int>> stack = {{x, y}};
        seen[i] = 1;
        while (!stack.empty()) {
          const auto [cx, cy] = stack.back();
          stack.pop_back();
          for (int k = 0; k < 4; ++k) {
            const int nx = cx + DX[k], ny = cy + DY[k];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            const size_t ni = static_cast<size_t>(ny) * w + nx;
            if (m[ni] && !seen[ni]) {
              seen[ni] = 1;
              stack.push_back({nx, ny});
            }
          }
        }
      }
    return count;
  }

  // 4-connected erosion of a 1-cell border (radius r in cells).
  static std::vector<uint8_t> erode(const std::vector<uint8_t>& m, int w,
                                    int h, int r) {
    std::vector<uint8_t> out(m.size(), 0);
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        if (!m[static_cast<size_t>(y) * w + x]) continue;
        bool keep = true;
        for (int dy = -r; dy <= r && keep; ++dy)
          for (int dx = -r; dx <= r && keep; ++dx) {
            const int nx = x + dx, ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h ||
                !m[static_cast<size_t>(ny) * w + nx])
              keep = false;
          }
        out[static_cast<size_t>(y) * w + x] = keep ? 1 : 0;
      }
    return out;
  }

  void mergeSmallRegions(std::vector<int16_t>& labels,
                         const std::vector<uint8_t>& free, int w, int h,
                         double res) const {
    const double minCells = p_.minAreaM2 / (res * res);
    bool changed = true;
    while (changed) {
      changed = false;
      // Find the smallest region.
      std::vector<int> counts(4096, 0);
      int maxId = 0;
      for (int16_t l : labels) {
        if (l <= 0) continue;
        counts[l]++;
        maxId = std::max(maxId, static_cast<int>(l));
      }
      if (static_cast<int>(counts.size()) <= maxId)
        counts.resize(static_cast<size_t>(maxId) + 1, 0);
      int small = -1, smallN = static_cast<int>(minCells);
      for (int id = 1; id <= maxId; ++id)
        if (counts[id] > 0 && counts[id] < smallN) {
          smallN = counts[id];
          small = id;
        }
      if (small < 0) break;
      // Absorb it into the neighbour label with the longest shared border.
      std::vector<std::pair<int, int>> tally;   // (neighbourId, sharedCells)
      auto bump = [&](int n) {
        for (auto& t : tally)
          if (t.first == n) { ++t.second; return; }
        tally.push_back({n, 1});
      };
      for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
          if (labels[static_cast<size_t>(y) * w + x] != small) continue;
          static const int DX[4] = {1, -1, 0, 0};
          static const int DY[4] = {0, 0, 1, -1};
          for (int k = 0; k < 4; ++k) {
            const int nx = x + DX[k], ny = y + DY[k];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            const int16_t n = labels[static_cast<size_t>(ny) * w + nx];
            if (n > 0 && n != small) bump(n);
          }
        }
      if (tally.empty()) {
        // Orphan speck: just delete it.
        for (int16_t& l : labels)
          if (l == small) l = 0;
      } else {
        const auto best = std::max_element(
            tally.begin(), tally.end(),
            [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
              return a.second < b.second;
            });
        for (int16_t& l : labels)
          if (l == small) l = static_cast<int16_t>(best->first);
      }
      changed = true;
    }
  }
};

}  // namespace vacuum
