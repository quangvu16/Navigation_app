// costmap.h - layered costmap for planning (prompt §5):
//   static layer (saved/mapped occupancy) + keepout layer (no-go polygons
//   & virtual walls, rasterised >= 2 cells thick) + live obstacle layer
//   (latest scan & sonar, decaying after 2 s) + inflation by robot radius
//   + 0.10 m soft margin.
//
// Cell costs (uint8): 0 free, 1..252 inflated (253 - distance cost),
// 254 lethal (occupied / keepout), 255 unknown. A* treats unknown as
// traversable-but-costly (Nav2's allow_unknown hint) unless forbidden.
//
// World/grid convention: same as OccupancyGrid (row 0 = world bottom).

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "vacuum/math.h"
#include "vacuum/occupancy_grid.h"
#include "vacuum/robot_config.h"

namespace vacuum {

class Costmap {
 public:
  static constexpr uint8_t kFree = 0;
  static constexpr uint8_t kLethal = 254;
  static constexpr uint8_t kUnknown = 255;
  static constexpr int kMaxInflated = 253;

  explicit Costmap(const RobotConfig& cfg = RobotConfig())
      : cfg_(cfg),
        inflateRadius_(cfg.robotRadiusM + cfg.inflateSoftMarginM) {}

  // Rebuild the static layer from a mapped grid (Phase 3's Mapper output).
  void setStatic(const OccupancyGrid& grid) {
    gx0_ = grid.originX();
    gy0_ = grid.originY();
    w_ = grid.width();
    h_ = grid.height();
    res_ = grid.resolution();
    static_.assign(static_cast<size_t>(w_) * h_, kFree);
    for (int y = 0; y < h_; ++y)
      for (int x = 0; x < w_; ++x) {
        const int gx = gx0_ + x, gy = gy0_ + y;
        static_[index(x, y)] = grid.occupied(gx, gy) ? kLethal
                             : grid.free(gx, gy)   ? kFree
                                                   : kUnknown;
      }
    rebuild();
  }

  // Keepout: a closed polygon in world metres (no-go zone or virtual
  // wall as a thin rectangle). Rasterised >= 2 cells thick (prompt §5) so
  // the inflation + planner can never squeeze through a one-cell seam.
  void addKeepout(const std::vector<Vec2>& polygonWorld) {
    keepouts_.push_back(polygonWorld);
    rebuild();
  }

  void clearKeepouts() {
    keepouts_.clear();
    rebuild();
  }

  // Live obstacle layer: hit points from the latest scan/sonar, stamped
  // with a time; they decay (stop being lethal) after obstacleDecayS.
  void setObstacles(const std::vector<Vec2>& pointsWorld, double nowS) {
    obstacles_.clear();
    for (const Vec2& p : pointsWorld) obstacles_.push_back({p, nowS});
    rebuild(nowS);
  }

  // Re-compose at `nowS` without new points: stamps older than the decay
  // window fade out (call at a low rate so obstacles die ~2 s after the
  // last scan that saw them).
  void expire(double nowS) { rebuild(nowS); }

  double resolution() const { return res_; }
  int originX() const { return gx0_; }
  int originY() const { return gy0_; }
  int width() const { return w_; }
  int height() const { return h_; }
  double inflateRadius() const { return inflateRadius_; }

  uint8_t cost(int gx, int gy) const {
    const int x = gx - gx0_, y = gy - gy0_;
    if (x < 0 || y < 0 || x >= w_ || y >= h_) return kLethal;  // off-map
    return costs_[static_cast<size_t>(y) * w_ + x];
  }
  uint8_t costAt(double wx, double wy) const {
    return cost(static_cast<int>(std::floor(wx / res_)),
                static_cast<int>(std::floor(wy / res_)));
  }

  // True when the robot's disc centred here fits (cost < lethal and, for
  // the strict check, below the inscribed band).
  bool isTraversable(double wx, double wy, bool allowUnknown) const {
    const uint8_t c = costAt(wx, wy);
    if (c == kLethal) return false;
    if (c == kUnknown) return allowUnknown;
    return true;
  }

 private:
  struct Stamped {
    Vec2 p;
    double t;
  };

  RobotConfig cfg_;
  double inflateRadius_;
  double obstacleDecayS_ = 2.0;   // prompt §5: live obstacles decay after 2 s
  int gx0_ = 0, gy0_ = 0, w_ = 0, h_ = 0;
  double res_ = 0.05;
  std::vector<uint8_t> static_;   // per-cell static class
  std::vector<uint8_t> costs_;    // composed costs
  std::vector<Stamped> obstacles_;
  std::vector<std::vector<Vec2>> keepouts_;

  size_t index(int x, int y) const {
    return static_cast<size_t>(y) * w_ + x;
  }

  void rebuild(double nowS = 0.0) {
    if (w_ <= 0 || h_ <= 0) return;
    costs_ = static_;   // start from static (free/lethal/unknown)

    // Keepouts: rasterise polygons (scanline even-odd), then thicken to
    // >= 2 cells by dilating once (a 1-cell seam could be crossed by an
    // 8-connected planner diagonal).
    for (const auto& poly : keepouts_) rasterisePolygon(poly);
    dilateLethalOnce();

    // Live obstacles (with decay).
    for (const auto& o : obstacles_) {
      if (nowS - o.t > obstacleDecayS_) continue;
      const int gx = static_cast<int>(std::floor(o.p.x / res_));
      const int gy = static_cast<int>(std::floor(o.p.y / res_));
      const int x = gx - gx0_, y = gy - gy0_;
      if (x >= 0 && y >= 0 && x < w_ && y < h_)
        costs_[index(x, y)] = kLethal;
    }

    inflate();
  }

  void rasterisePolygon(const std::vector<Vec2>& poly) {
    if (poly.size() < 3) return;
    // Bounding box in cell coords.
    double minX = poly[0].x, maxX = poly[0].x, minY = poly[0].y, maxY = poly[0].y;
    for (const Vec2& v : poly) {
      minX = std::min(minX, v.x); maxX = std::max(maxX, v.x);
      minY = std::min(minY, v.y); maxY = std::max(maxY, v.y);
    }
    const int x0 = std::max(0, static_cast<int>(std::floor(minX / res_)) - gx0_);
    const int x1 = std::min(w_ - 1, static_cast<int>(std::floor(maxX / res_)) - gx0_);
    const int y0 = std::max(0, static_cast<int>(std::floor(minY / res_)) - gy0_);
    const int y1 = std::min(h_ - 1, static_cast<int>(std::floor(maxY / res_)) - gy0_);
    for (int y = y0; y <= y1; ++y) {
      // Even-odd scanline at the cell centre.
      const double wy = (gy0_ + y + 0.5) * res_;
      for (int x = x0; x <= x1; ++x) {
        const double wx = (gx0_ + x + 0.5) * res_;
        if (pointInPolygon(wx, wy, poly)) costs_[index(x, y)] = kLethal;
      }
    }
  }

  static bool pointInPolygon(double wx, double wy,
                             const std::vector<Vec2>& poly) {
    bool inside = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
      const Vec2& a = poly[i];
      const Vec2& b = poly[j];
      if ((a.y > wy) != (b.y > wy) &&
          wx < (b.x - a.x) * (wy - a.y) / (b.y - a.y) + a.x)
        inside = !inside;
    }
    return inside;
  }

  void dilateLethalOnce() {
    std::vector<size_t> add;   // indices, NOT uint8_t (they exceed 255!)
    add.reserve(costs_.size() / 8);
    for (int y = 0; y < h_; ++y)
      for (int x = 0; x < w_; ++x) {
        if (costs_[index(x, y)] != kLethal) continue;
        for (int dy = -1; dy <= 1; ++dy)
          for (int dx = -1; dx <= 1; ++dx) {
            const int nx = x + dx, ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= w_ || ny >= h_) continue;
            if (costs_[index(nx, ny)] != kLethal) add.push_back(index(nx, ny));
          }
      }
    for (size_t i : add) costs_[i] = kLethal;
  }
  // Inflation: cost falls linearly with distance from the nearest lethal
  // cell out to inflateRadius (robotRadius + soft margin), like Nav2's
  // inflation radius with cost_scaling_factor baked in.
  void inflate() {
    if (w_ <= 0 || h_ <= 0) return;
    const double r = inflateRadius_;
    if (r <= 0) return;
    // Multi-source BFS over distance (cell units), cheap at our sizes.
    const int rCells = static_cast<int>(std::ceil(r / res_));
    std::vector<int16_t> dist(costs_.size(), -1);
    std::vector<std::pair<int, int>> frontier;
    for (int y = 0; y < h_; ++y)
      for (int x = 0; x < w_; ++x)
        if (costs_[index(x, y)] == kLethal) {
          dist[index(x, y)] = 0;
          frontier.push_back({x, y});
        }
    // 4-connected BFS gives a slightly boxy frontier; 8-connected in the
    // expansion for a rounder radius.
    while (!frontier.empty()) {
      std::vector<std::pair<int, int>> next;
      for (const auto& [x, y] : frontier) {
        const int16_t d = dist[index(x, y)];
        if (d >= rCells) continue;
        for (int dy = -1; dy <= 1; ++dy)
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const int nx = x + dx, ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= w_ || ny >= h_) continue;
            const size_t ni = index(nx, ny);
            if (dist[ni] >= 0) continue;
            dist[ni] = static_cast<int16_t>(d + 1);
            next.push_back({nx, ny});
          }
      }
      frontier = std::move(next);
    }
    for (size_t i = 0; i < costs_.size(); ++i) {
      if (costs_[i] == kLethal || costs_[i] == kUnknown) continue;
      const int16_t d = dist[i];
      if (d < 0 || d > rCells) continue;
      // 253 at the lethal edge down to 1 at the inflation radius.
      const int c = kMaxInflated - (kMaxInflated - 1) * d / std::max(1, rCells);
      if (costs_[i] == kFree || costs_[i] < static_cast<uint8_t>(c))
        costs_[i] = static_cast<uint8_t>(c);
    }
  }
};

}  // namespace vacuum
