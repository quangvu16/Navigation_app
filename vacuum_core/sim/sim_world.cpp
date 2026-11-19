// sim_world.cpp — grid DDA raycaster. See sim_world.h for conventions.

#include "sim_world.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace vacuum {

SimWorld::SimWorld(int width, int height, double resolution, Vec2 origin,
                   const uint8_t* cells)
    : width_(width), height_(height), resolution_(resolution), origin_(origin) {
  cells_.assign(cells, cells + static_cast<size_t>(width) * height);
}

SimWorld SimWorld::emptyRoom(double sizeM, double resolution,
                             double wallThicknessM) {
  const int n = static_cast<int>(std::ceil(sizeM / resolution));
  std::vector<uint8_t> cells(static_cast<size_t>(n) * n, kFree);
  const int wallCells = static_cast<int>(std::ceil(wallThicknessM / resolution));
  for (int cy = 0; cy < n; ++cy) {
    for (int cx = 0; cx < n; ++cx) {
      const bool wall = cx < wallCells || cy < wallCells ||
                        cx >= n - wallCells || cy >= n - wallCells;
      if (wall) cells[cy * n + cx] = kOccupied;
    }
  }
  return SimWorld(n, n, resolution, Vec2(0, 0), cells.data());
}

SimWorld SimWorld::loadPgm(const std::string& path, double resolution,
                           Vec2 origin, uint8_t freeBelow) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("SimWorld::loadPgm: cannot open " + path);
  std::string magic;
  in >> magic;
  if (magic != "P5") throw std::runtime_error("SimWorld::loadPgm: not a P5 file");
  // Skip comments.
  auto skipWsAndComments = [&] {
    for (;;) {
      int c = in.peek();
      if (c == '#') {
        std::string line;
        std::getline(in, line);
      } else if (std::isspace(c)) {
        in.get();
      } else {
        break;
      }
    }
  };
  skipWsAndComments();
  int w = 0, h = 0, maxval = 0;
  in >> w;
  skipWsAndComments();
  in >> h;
  skipWsAndComments();
  in >> maxval;
  in.get();  // single whitespace after maxval
  if (w <= 0 || h <= 0 || maxval <= 0 || maxval > 255)
    throw std::runtime_error("SimWorld::loadPgm: bad header");
  std::vector<uint8_t> gray(static_cast<size_t>(w) * h);
  in.read(reinterpret_cast<char*>(gray.data()), static_cast<std::streamsize>(gray.size()));
  if (!in) throw std::runtime_error("SimWorld::loadPgm: truncated data");

  std::vector<uint8_t> cells(gray.size());
  for (size_t i = 0; i < gray.size(); ++i)
    cells[i] = gray[i] < freeBelow ? kOccupied : kFree;
  // PGM row 0 is the TOP of the image; flip so row 0 = bottom (grid
  // convention). Mirrors what the app's Bitmap->grid path does with PNGs.
  std::vector<uint8_t> flipped(cells.size());
  for (int y = 0; y < h; ++y)
    std::memcpy(&flipped[static_cast<size_t>(y) * w],
                &cells[static_cast<size_t>(h - 1 - y) * w], w);
  return SimWorld(w, h, resolution, origin, flipped.data());
}

double SimWorld::raycast(Vec2 origin, double angleRad, double maxRangeM) const {
  return raycast(origin, angleRad, maxRangeM, nullptr);
}

// Grid DDA (Amanatides & Woo): walks cell boundaries exactly, so thin walls
// can never be stepped over. Out-of-grid counts as a hit at the border —
// the world is a closed box from the ray's point of view.
double SimWorld::raycast(Vec2 origin, double angleRad, double maxRangeM,
                         uint8_t* intensity) const {
  if (width_ == 0 || height_ == 0) return maxRangeM;

  const double dx = std::cos(angleRad), dy = std::sin(angleRad);
  // Grid coordinates (cell units). origin_ is the world position of cell (0,0)'s
  // bottom-left corner.
  const double fx = (origin.x - origin_.x) / resolution_;
  const double fy = (origin.y - origin_.y) / resolution_;
  int cx = static_cast<int>(std::floor(fx));
  int cy = static_cast<int>(std::floor(fy));

  // Outside the grid entirely: distance to the grid border along the ray.
  auto borderDistance = [&]() -> double {
    double t = maxRangeM;
    if (dx != 0.0) {
      const double bx = (dx > 0.0) ? (width_ - fx) : -fx;
      const double cand = bx * resolution_ / dx;
      if (cand >= 0.0 && cand < t) t = cand;
    }
    if (dy != 0.0) {
      const double by = (dy > 0.0) ? (height_ - fy) : -fy;
      const double cand = by * resolution_ / dy;
      if (cand >= 0.0 && cand < t) t = cand;
    }
    return t < 0.0 ? 0.0 : t;
  };

  if (!inBounds(cx, cy)) return borderDistance();

  const int stepX = (dx > 0.0) ? 1 : (dx < 0.0 ? -1 : 0);
  const int stepY = (dy > 0.0) ? 1 : (dy < 0.0 ? -1 : 0);

  // Distance (in metres along the ray) to the next cell boundary.
  const double inf = 1e18;
  const double tDeltaX = (dx != 0.0) ? std::abs(resolution_ / dx) : inf;
  const double tDeltaY = (dy != 0.0) ? std::abs(resolution_ / dy) : inf;
  double tMaxX = inf, tMaxY = inf;
  if (stepX > 0) tMaxX = (cx + 1 - fx) * resolution_ / dx;
  else if (stepX < 0) tMaxX = (cx - fx) * resolution_ / dx;
  if (stepY > 0) tMaxY = (cy + 1 - fy) * resolution_ / dy;
  else if (stepY < 0) tMaxY = (cy - fy) * resolution_ / dy;

  double t = 0.0;
  for (;;) {
    if (cell(cx, cy) != kFree) {
      // Hit the cell we are standing in (e.g. started inside a wall).
      if (intensity) *intensity = 0;
      return t;
    }
    // Advance to the next boundary.
    if (tMaxX < tMaxY) {
      t = tMaxX;
      tMaxX += tDeltaX;
      cx += stepX;
    } else {
      t = tMaxY;
      tMaxY += tDeltaY;
      cy += stepY;
    }
    if (t > maxRangeM) {
      if (intensity) *intensity = 0;
      return maxRangeM;
    }
    if (!inBounds(cx, cy)) {
      if (intensity) *intensity = 0;
      return std::min(t, maxRangeM);
    }
    if (cell(cx, cy) != kFree) {
      if (intensity) {
        // Synthetic reflectivity: near hits read bright, far hits dim, plus a
        // stable per-cell jitter so consecutive scans look sensor-like.
        const uint32_t hash = (static_cast<uint32_t>(cx) * 73856093u) ^
                              (static_cast<uint32_t>(cy) * 19349663u);
        const double fall = std::exp(-t / 3.0);
        int v = static_cast<int>(200.0 * fall) + static_cast<int>(hash % 40u);
        *intensity = static_cast<uint8_t>(v > 255 ? 255 : v);
      }
      return t;
    }
  }
}

}  // namespace vacuum
