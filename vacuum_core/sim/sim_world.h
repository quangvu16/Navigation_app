// sim_world.h — the simulated floor: an occupancy grid with grid-DDA
// raycasting, used by the lidar and sonar models.
//
// Grid convention matches the map model (docs/ARCHITECTURE.md): cells are
// row-major, row 0 is the BOTTOM of the world (y = origin.y), so
// cell(x, y) index = cy * width + cx with cy measured up. This is the
// "grid origin bottom-left" convention from the prompt; the renderer flips
// Y only at draw time.
//
// Cell values: 0 = free, 1 = occupied, 2 = unknown. Unknown blocks rays
// (fails safe: the sim never lets the robot see through unexplored walls).

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "vacuum/math.h"

namespace vacuum {

class SimWorld {
 public:
  enum Cell : uint8_t { kFree = 0, kOccupied = 1, kUnknown = 2 };

  SimWorld() = default;

  // cells: width*height bytes, row-major, row 0 = bottom. Copied in.
  SimWorld(int width, int height, double resolution, Vec2 origin,
           const uint8_t* cells);

  // Convenience: a rectangular room with four walls, `sizeM` on a side,
  // 0.05 m cells (same as the SLAM grid). Great for unit tests and the
  // default demo scene.
  static SimWorld emptyRoom(double sizeM, double resolution = 0.05,
                            double wallThicknessM = 0.10);

  // Host-side loader for a binary PGM (P5); black (0) = occupied, anything
  // above `freeBelow` = free. Used by vacuum_core tools/tests. The Android
  // app decodes PNG floor plans with android.graphics.Bitmap and passes the
  // grid through fromGrid() instead (no image dependency in the core).
  static SimWorld loadPgm(const std::string& path, double resolution,
                          Vec2 origin, uint8_t freeBelow = 128);

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  Vec2 origin() const { return origin_; }
  const std::vector<uint8_t>& cells() const { return cells_; }

  bool inBounds(int cx, int cy) const {
    return cx >= 0 && cy >= 0 && cx < width_ && cy < height_;
  }
  uint8_t cell(int cx, int cy) const {
    return inBounds(cx, cy) ? cells_[cy * width_ + cx] : kUnknown;
  }
  // World position -> cell; out-of-grid counts as unknown/occupied.
  uint8_t cellAt(double wx, double wy) const {
    return cell(static_cast<int>((wx - origin_.x) / resolution_),
                static_cast<int>((wy - origin_.y) / resolution_));
  }
  bool isOccupied(double wx, double wy) const {
    return cellAt(wx, wy) != kFree;
  }

  // Cast a ray from `origin` at `angleRad` (world frame, CCW from +x) up to
  // maxRangeM. Returns the distance to the first blocked cell, or maxRangeM
  // when nothing is hit. Uses a DDA over cell boundaries (exact for grids —
  // no step-size tuning, no missed thin walls).
  double raycast(Vec2 origin, double angleRad, double maxRangeM) const;

  // Raycast that also reports a synthetic reflectivity for the hit cell:
  // bright up close, falling off with range, plus a per-cell hash so a
  // reflective strip painted as high-valued cells reads as "hot bins" —
  // the effect the dock detector will use (LDS2 intensity, docs/PROTOCOL.md).
  // `intensity` is written only when the ray hits a cell before maxRange.
  double raycast(Vec2 origin, double angleRad, double maxRangeM,
                 uint8_t* intensity) const;

 private:
  int width_ = 0, height_ = 0;
  double resolution_ = 0.05;
  Vec2 origin_;
  std::vector<uint8_t> cells_;
};

}  // namespace vacuum
