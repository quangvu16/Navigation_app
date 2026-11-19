// occupancy_grid.h - log-odds occupancy grid with dynamic growth.
//
// Model (prompt §5): 0.05 m cells, log-odds l in [-l_max, l_max];
// unknown = never updated (l == 0 exactly, the prior), free = l < 0,
// occupied = l > 0. Cells grow in tiles of 64x64 when a measurement lands
// outside the current bounds - a map that must be sized upfront wastes
// memory and fails on open plans.
//
// Row-major, row 0 = BOTTOM of the world (grid origin bottom-left, same
// convention as SimWorld and the renderer's flip). The grid's (gx0, gy0) is
// the cell coordinate of the array's [0] in an unbounded integer grid, so
// growth at the top/left just shifts the origin.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "vacuum/math.h"

namespace vacuum {

class OccupancyGrid {
 public:
  static constexpr int kTile = 64;          // growth granularity (prompt §5)
  static constexpr float kLOddMin = -4.0f;  // clamp: confident free
  static constexpr float kLOddMax = 4.0f;   // clamp: confident occupied
  static constexpr float kHitDelta = 0.85f;
  static constexpr float kMissDelta = -0.4f;
  static constexpr float kOccupiedTh = 0.5f;  // l > this = obstacle
  static constexpr float kFreeTh = -0.5f;     // l < this = definitely free

  explicit OccupancyGrid(double resolution = 0.05)
      : resolution_(resolution) {
    // Start with one tile at the origin cell (0,0)-(63,63).
    allocate(0, 0, kTile, kTile);
  }

  double resolution() const { return resolution_; }
  int width() const { return w_; }
  int height() const { return h_; }
  int originX() const { return gx0_; }   // cell coord of column 0
  int originY() const { return gy0_; }   // cell coord of row 0

  // World position of a cell's centre.
  Vec2 cellCentre(int gx, int gy) const {
    return {originX() * resolution_ + (gx - originX() + 0.5) * resolution_,
            originY() * resolution_ + (gy - originY() + 0.5) * resolution_};
  }
  // Cell containing a world point (may be outside the current bounds).
  void worldToCell(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor(wx / resolution_));
    gy = static_cast<int>(std::floor(wy / resolution_));
  }

  float logOdds(int gx, int gy) const {
    if (!inBounds(gx, gy)) return 0.0f;   // outside = unknown prior
    return cells_[index(gx, gy)];
  }
  bool occupied(int gx, int gy) const { return logOdds(gx, gy) > kOccupiedTh; }
  bool free(int gx, int gy) const { return logOdds(gx, gy) < kFreeTh; }
  bool unknown(int gx, int gy) const {
    const float l = logOdds(gx, gy);
    return l >= kFreeTh && l <= kOccupiedTh;
  }

  void setLogOdds(int gx, int gy, float l) {
    ensure(gx, gy);
    cells_[index(gx, gy)] = clampLOdd(l);
  }

  // Increment one cell (clamped).
  void addLogOdds(int gx, int gy, float delta) {
    ensure(gx, gy);
    float& c = cells_[index(gx, gy)];
    c = clampLOdd(c + delta);
  }

  // Integrate one beam: free cells along the ray (miss deltas), hit cell at
  // the end (hit delta). The sensor cell itself is skipped. `hit` false =
  // no return: the ray only clears up to maxRange.
  void rayHit(Vec2 fromW, Vec2 toW) {
    integrateRay(fromW, toW, true);
  }
  void rayMiss(Vec2 fromW, Vec2 toW) {
    integrateRay(fromW, toW, false);
  }

  // Clear the whole map back to unknown (map restart).
  void clear() {
    std::fill(cells_.begin(), cells_.end(), 0.0f);
  }

  // ---- serialisation (prompt §6: grid.bin, RLE) ---------------------------
  // Layout: "VOG1" | int32 gx0 | int32 gy0 | int32 w | int32 h |
  // float resolution | RLE stream of (int8 quantised log-odds, u16 count).
  // Quantisation step 0.25 over [-4,4] -> 33 levels, fits int8.
  std::string serializeRle() const {
    std::string out("VOG1", 4);
    appendI32(out, gx0_);
    appendI32(out, gy0_);
    appendI32(out, w_);
    appendI32(out, h_);
    appendF32(out, static_cast<float>(resolution_));
    size_t i = 0;
    while (i < cells_.size()) {
      const int8_t q = quant(cells_[i]);
      size_t j = i + 1;
      while (j < cells_.size() && quant(cells_[j]) == q && (j - i) < 0xFFFF)
        ++j;
      out.push_back(static_cast<char>(q));
      const uint16_t n = static_cast<uint16_t>(j - i);
      out.push_back(static_cast<char>(n & 0xFF));
      out.push_back(static_cast<char>((n >> 8) & 0xFF));
      i = j;
    }
    return out;
  }

  static bool deserializeRle(const std::string& data, OccupancyGrid& out) {
    if (data.size() < 4 + 20 || data.compare(0, 4, "VOG1") != 0) return false;
    const uint8_t* p = reinterpret_cast<const uint8_t*>(data.data()) + 4;
    const int gx0 = readI32(p); p += 4;
    const int gy0 = readI32(p); p += 4;
    const int w = readI32(p); p += 4;
    const int h = readI32(p); p += 4;
    const float res = readF32(p); p += 4;
    if (w <= 0 || h <= 0 || w > 1 << 20 || h > 1 << 20) return false;
    const size_t need = static_cast<size_t>(w) * h;
    OccupancyGrid g(res);
    g.allocate(gx0, gy0, w, h);
    size_t i = 0;
    const uint8_t* end = reinterpret_cast<const uint8_t*>(data.data()) + data.size();
    while (i < need && p + 3 <= end) {
      const float l = unquant(static_cast<int8_t>(*p++));
      const uint16_t n = static_cast<uint16_t>(p[0] | (p[1] << 8));
      p += 2;
      for (uint16_t k = 0; k < n && i < need; ++k) g.cells_[i++] = l;
    }
    if (i != need) return false;
    out = std::move(g);
    return true;
  }

  const std::vector<float>& rawCells() const { return cells_; }

 private:
  double resolution_;
  int gx0_ = 0, gy0_ = 0, w_ = 0, h_ = 0;
  std::vector<float> cells_;

  bool inBounds(int gx, int gy) const {
    return gx >= gx0_ && gy >= gy0_ && gx < gx0_ + w_ && gy < gy0_ + h_;
  }
  size_t index(int gx, int gy) const {
    return static_cast<size_t>(gy - gy0_) * w_ + (gx - gx0_);
  }
  static float clampLOdd(float l) {
    return std::max(kLOddMin, std::min(kLOddMax, l));
  }

  void allocate(int gx0, int gy0, int w, int h) {
    gx0_ = gx0;
    gy0_ = gy0;
    w_ = w;
    h_ = h;
    cells_.assign(static_cast<size_t>(w) * h, 0.0f);
  }

  // Grow by whole tiles so (gx,gy) is inside; shifts gx0_/gy0_ when the
  // point is left/below the current origin (preserving cell coordinates).
  void ensure(int gx, int gy) {
    if (inBounds(gx, gy)) return;
    int nx0 = gx0_, ny0 = gy0_, nw = w_, nh = h_;
    if (gx < nx0) {
      const int grow = roundUp(nx0 - gx, kTile);
      nx0 -= grow;
      nw += grow;
    } else if (gx >= nx0 + nw) {
      nw += roundUp(gx - (nx0 + nw) + 1, kTile);
    }
    if (gy < ny0) {
      const int grow = roundUp(ny0 - gy, kTile);
      ny0 -= grow;
      nh += grow;
    } else if (gy >= ny0 + nh) {
      nh += roundUp(gy - (ny0 + nh) + 1, kTile);
    }
    // Re-seat the old cells into the new allocation (origin may have moved).
    std::vector<float> fresh(static_cast<size_t>(nw) * nh, 0.0f);
    for (int y = 0; y < h_; ++y)
      std::memcpy(&fresh[static_cast<size_t>(y + (gy0_ - ny0_)) * nw +
                         (gx0_ - nx0_)],
                  &cells_[static_cast<size_t>(y) * w_],
                  static_cast<size_t>(w_) * sizeof(float));
    cells_ = std::move(fresh);
    gx0_ = nx0;
    gy0_ = ny0;
    w_ = nw;
    h_ = nh;
  }

  static int roundUp(int v, int m) { return ((v + m - 1) / m) * m; }

  static int8_t quant(float l) {
    // [-4,4] step 0.25 -> [-16,16] offset by 16 -> [0,32] fits int8.
    const int q = static_cast<int>(std::lround(clampLOdd(l) * 4.0f)) + 16;
    return static_cast<int8_t>(std::max(0, std::min(32, q)));
  }
  static float unquant(int8_t q) {
    return (static_cast<float>(q) - 16.0f) / 4.0f;
  }

  static void appendI32(std::string& s, int32_t v) {
    for (int i = 0; i < 4; ++i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
  }
  static void appendF32(std::string& s, float f) {
    int32_t v;
    std::memcpy(&v, &f, 4);
    appendI32(s, v);
  }
  static int32_t readI32(const uint8_t* p) {
    return static_cast<int32_t>(static_cast<uint32_t>(p[0]) |
                                (static_cast<uint32_t>(p[1]) << 8) |
                                (static_cast<uint32_t>(p[2]) << 16) |
                                (static_cast<uint32_t>(p[3]) << 24));
  }
  static float readF32(const uint8_t* p) {
    const int32_t v = readI32(p);
    float f;
    std::memcpy(&f, &v, 4);
    return f;
  }

  // Bresenham-style DDA for beam integration (shared by rayHit/rayMiss).
  void integrateRay(Vec2 fromW, Vec2 toW, bool hit) {
    int x0, y0, x1, y1;
    worldToCell(fromW.x, fromW.y, x0, y0);
    worldToCell(toW.x, toW.y, x1, y1);
    // Straight-line walk; the last cell gets the hit delta (or a miss too
    // when !hit - the endpoint of a no-return ray is also cleared).
    int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
    const int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int x = x0, y = y0;
    for (;;) {
      const bool last = (x == x1 && y == y1);
      if (!(x == x0 && y == y0)) {          // never clear the sensor cell
        if (last && hit)
          addLogOdds(x, y, kHitDelta);
        else
          addLogOdds(x, y, kMissDelta);
      } else if (!hit && last) {
        // degenerate: sensor cell == endpoint, nothing to do
      }
      if (last) break;
      const int e2 = 2 * err;
      if (e2 >= dy) { err += dy; x += sx; }
      if (e2 <= dx) { err += dx; y += sy; }
    }
  }
};

}  // namespace vacuum
