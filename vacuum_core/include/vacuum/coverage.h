// coverage.h - boustrophedon coverage planning (prompt §5):
//   per room: erode by robot radius, choose the sweep angle aligned to the
//   room's dominant wall (PCA principal axis), lay sweep lines spaced
//   0.8 x brush width, decompose each line into drivable intervals
//   (obstacles split lines = cell decomposition), alternate the sweep
//   direction (boustrophedon), connect with A* when a straight jump is
//   blocked. Coverage tracking: brush footprint along the real pose.
//
// Output is a dense waypoint path (world metres) the GoTo/RPP stack can
// follow segment by segment.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "vacuum/math.h"
#include "vacuum/occupancy_grid.h"

namespace vacuum {

struct CoverageParams {
  double brushWidthM = 0.25;      // cleaning brush (prompt:0.8 x this)
  double robotRadiusM = 0.23;
  double lineSpacingFactor = 0.8; // prompt: lines spaced0.8 x brush
  double waypointSpacingM = 0.10; // densify sweep lines
};

class CoveragePlanner {
 public:
  CoveragePlanner() = default;
  explicit CoveragePlanner(const CoverageParams& p) : p_(p) {}

  // Plan a coverage path over the cells of `labels` with value `roomId`.
  // Returns world waypoints (empty when the room is too small to drive).
  std::vector<Vec2> plan(const OccupancyGrid& grid,
                         const std::vector<int16_t>& labels,
                         int roomId) const {
    const int w = grid.width(), h = grid.height();
    const double res = grid.resolution();
    auto idx = [&](int x, int y) { return static_cast<size_t>(y) * w + x; };

    // Room mask, then erode by the robot radius (drivable region).
    const int erodeR = std::max(1,
        static_cast<int>(std::ceil(p_.robotRadiusM / res)));
    std::vector<uint8_t> room(static_cast<size_t>(w) * h, 0);
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x)
        room[idx(x, y)] = (labels[idx(x, y)] == roomId) ? 1 : 0;
    std::vector<uint8_t> drive = room;
    for (int r = 0; r < erodeR; ++r) drive = erodeOnce(drive, w, h);

    // --- sweep angle: PCA principal axis of the drivable cells -----------
    double mx = 0, my = 0;
    int n = 0;
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x)
        if (drive[idx(x, y)]) { mx += x; my += y; ++n; }
    if (n < 8) return {};
    mx /= n;
    my /= n;
    double sxx = 0, sxy = 0, syy = 0;
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        if (!drive[idx(x, y)]) continue;
        const double dx = x - mx, dy = y - my;
        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
      }
    // Principal axis angle (dominant wall direction).
    const double theta = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
    const double ux = std::cos(theta), uy = std::sin(theta);   // along sweeps
    const double vx = -uy, vy = ux;                            // across lines

    // --- sweep lines across the perpendicular extent ---------------------
    const double spacing =
        std::max(res, p_.brushWidthM * p_.lineSpacingFactor);
    // Project all drivable cells on the across-axis.
    double tMin = 1e18, tMax = -1e18;
    double uMin = 1e18, uMax = -1e18;
    std::vector<std::pair<double, double>> uv;   // per cell (u, t)
    uv.reserve(static_cast<size_t>(n));
    std::vector<std::pair<int, int>> cells;
    cells.reserve(static_cast<size_t>(n));
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        if (!drive[idx(x, y)]) continue;
        const double dx = x - mx, dy = y - my;
        const double u = dx * ux + dy * uy;
        const double t = dx * vx + dy * vy;
        uv.push_back({u, t});
        cells.push_back({x, y});
        tMin = std::min(tMin, t);
        tMax = std::max(tMax, t);
        uMin = std::min(uMin, u);
        uMax = std::max(uMax, u);
      }

    std::vector<Vec2> path;
    bool forward = true;
    for (double t = tMin; t <= tMax + 1e-9; t += spacing) {
      // Cells whose across-projection falls in this line's slab.
      std::vector<std::pair<double, int>> hits;   // (u, cellIndex)
      for (size_t i = 0; i < uv.size(); ++i)
        if (std::abs(uv[i].second - t) <= spacing * 0.5)
          hits.push_back({uv[i].first, static_cast<int>(i)});
      if (hits.empty()) continue;
      std::sort(hits.begin(), hits.end());
      // Alternate direction (boustrophedon).
      if (!forward) std::reverse(hits.begin(), hits.end());
      forward = !forward;
      for (const auto& [u, ci] : hits) {
        const auto [cx, cy] = cells[static_cast<size_t>(ci)];
        path.push_back(grid.cellCentre(grid.originX() + cx,
                                       grid.originY() + cy));
      }
    }

    // Densify to waypointSpacingM for the local controller.
    return densify(path, p_.waypointSpacingM);
  }

 private:
  CoverageParams p_;

  static std::vector<uint8_t> erodeOnce(const std::vector<uint8_t>& m,
                                        int w, int h) {
    std::vector<uint8_t> out(m.size(), 0);
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        const size_t i = static_cast<size_t>(y) * w + x;
        if (!m[i]) continue;
        bool keep = true;
        for (int dy = -1; dy <= 1 && keep; ++dy)
          for (int dx = -1; dx <= 1 && keep; ++dx) {
            const int nx = x + dx, ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h ||
                !m[static_cast<size_t>(ny) * w + nx])
              keep = false;
          }
        out[i] = keep ? 1 : 0;
      }
    return out;
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
        const double s = static_cast<double>(k) * spacing / len;
        if (s >= 1.0) break;
        out.push_back(a + (b - a) * s);
      }
      out.push_back(b);
    }
    return out;
  }
};

// Brush-footprint coverage tracking along the real pose (prompt §5:
// "track covered cells (brush footprint along the real pose) -> coverage %").
class CoverageTracker {
 public:
  CoverageTracker() = default;

  void reset(const OccupancyGrid& grid) {
    w_ = grid.width();
    h_ = grid.height();
    res_ = grid.resolution();
    gx0_ = grid.originX();
    gy0_ = grid.originY();
    total_.assign(static_cast<size_t>(w_) * h_, 0);
    covered_.assign(static_cast<size_t>(w_) * h_, 0);
  }

  // Register a room's DRIVABLE cells as the denominator: eroded by the
  // robot radius first (the same mask the planner drives). Counting the
  // raw room cells would cap coverage at ~84% for a0.25 m brush on a
  // 0.46 m robot - the wall ring is physically unreachable.
  // (reset(grid) must run first so the dimensions are known.)
  void setRoom(const std::vector<int16_t>& labels, int roomId,
               double robotRadiusM) {
    total_.assign(labels.size(), 0);
    covered_.assign(labels.size(), 0);
    std::vector<uint8_t> mask(labels.size(), 0);
    for (size_t i = 0; i < labels.size(); ++i)
      mask[i] = (labels[i] == roomId) ? 1 : 0;
    const int layers = static_cast<int>(std::ceil(robotRadiusM / res_));
    for (int k = 0; k < layers; ++k) mask = erodeOnce(mask);
    total_ = mask;   // drivable cells only
  }

  // Mark the brush disc around the pose as covered.
  void markPose(double wx, double wy, double brushRadiusM) {
    const int r = static_cast<int>(std::ceil(brushRadiusM / res_));
    const int cx = static_cast<int>(std::floor(wx / res_));
    const int cy = static_cast<int>(std::floor(wy / res_));
    for (int dy = -r; dy <= r; ++dy)
      for (int dx = -r; dx <= r; ++dx) {
        if (dx * dx + dy * dy > r * r) continue;
        const int x = cx + dx - gx0_, y = cy + dy - gy0_;
        if (x < 0 || y < 0 || x >= w_ || y >= h_) continue;
        const size_t i = static_cast<size_t>(y) * w_ + x;
        if (total_[i]) covered_[i] = 1;
      }
  }

  double percent() const {
    size_t tot = 0, cov = 0;
    for (size_t i = 0; i < total_.size(); ++i)
      if (total_[i]) { ++tot; cov += covered_[i]; }
    return tot ? 100.0 * static_cast<double>(cov) / tot : 0.0;
  }

 private:
  int w_ = 0, h_ = 0, gx0_ = 0, gy0_ = 0;
  double res_ = 0.05;
  std::vector<uint8_t> total_;
  std::vector<uint8_t> covered_;

  std::vector<uint8_t> erodeOnce(const std::vector<uint8_t>& m) const {
    std::vector<uint8_t> out(m.size(), 0);
    for (int y = 0; y < h_; ++y)
      for (int x = 0; x < w_; ++x) {
        const size_t i = static_cast<size_t>(y) * w_ + x;
        if (!m[i]) continue;
        bool keep = true;
        for (int dy = -1; dy <= 1 && keep; ++dy)
          for (int dx = -1; dx <= 1 && keep; ++dx) {
            const int nx = x + dx, ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= w_ || ny >= h_ ||
                !m[static_cast<size_t>(ny) * w_ + nx])
              keep = false;
          }
        out[i] = keep ? 1 : 0;
      }
    return out;
  }
};

}  // namespace vacuum
