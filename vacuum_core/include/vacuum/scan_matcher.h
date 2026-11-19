// scan_matcher.h - scan pre-processing + real-time correlative scan
// matching (Olson 2009) with a Gauss-Newton tail (prompt §5).
//
// Pre-processing: drop ranges < lidarMinRangeM (robot body) or > lidarMaxRangeM,
// drop bins inside the configurable body mask, DE-SKEW using the EKF motion
// during the revolution (each beam gets the pose it was actually taken at -
// LDS2's start/end ms make this possible), voxel-downsample to <= 0.03 m.
//
// Matching: maximise the sum of grid log-odds under the transformed points.
// Multi-resolution: a coarse pass scores against a 0.2 m strided view of the
// 0.05 m grid (smooths the surface so odometry seeding cannot fall into a
// one-cell local optimum), then a fine pass on the true grid, then a few
// Gauss-Newton steps on the score (finite-difference gradients).

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "vacuum/math.h"
#include "vacuum/occupancy_grid.h"
#include "vacuum/robot_config.h"
#include "vacuum/scan_transform.h"

namespace vacuum {

// ---------------------------------------------------------------- preprocessing
struct ScanPreprocessor {
  RobotConfig cfg;
  double voxelM = 0.03;  // prompt §5: downsample to <= 3 cm

  // ranges: wire convention (bin i clockwise from zero, 0 = no return).
  // poseStart/poseEnd: base pose at revolution start/end (board-time
  // aligned) for de-skewing. Returns points in the BASE frame at poseEnd.
  std::vector<Vec2> process(const float* ranges, int bins,
                            const Pose2D& poseStart,
                            const Pose2D& poseEnd) const {
    std::vector<Vec2> pts;
    pts.reserve(bins);
    const double span = 1.0 / 6.0;  // revolution duration; fraction = i/bins
    for (int i = 0; i < bins; ++i) {
      const double r = ranges[i];
      if (r <= 0.0 || r < cfg.lidarMinRangeM || r > cfg.lidarMaxRangeM)
        continue;
      // Beam direction in the base frame (mirror + mount yaw, one place).
      const double baseAng = binToBaseAngle(cfg, i, bins);
      // De-skew: the pose when beam i was taken (fraction i/bins through
      // the revolution - LDS2 acquires bins in order).
      const double f = static_cast<double>(i) / bins;
      const Pose2D pose(
          poseStart.x + f * (poseEnd.x - poseStart.x),
          poseStart.y + f * (poseEnd.y - poseStart.y),
          poseStart.theta + f * normalizeAngle(poseEnd.theta - poseStart.theta));
      // Point in the world, then back into the poseEnd base frame.
      const Vec2 origin = laserOrigin(cfg, pose);
      const double ang = pose.theta + deg2rad(cfg.lidarMountYawDeg) + baseAng;
      const Vec2 world = origin + Vec2(std::cos(ang), std::sin(ang)) * r;
      const Vec2 rel = rotate(world - poseEnd.translation(), -poseEnd.theta);
      pts.push_back(rel);
    }
    return voxelDownsample(pts, voxelM);
  }

  // Grid-hash voxel downsample: keep the first point per 3 cm cell.
  static std::vector<Vec2> voxelDownsample(const std::vector<Vec2>& pts,
                                           double voxel) {
    if (voxel <= 0.0 || pts.empty()) return pts;
    std::vector<Vec2> out;
    out.reserve(pts.size());
    // Small hash set: open addressing over int64 keys.
    struct Slot { int64_t key; bool used; };
    std::vector<Slot> table(1024, {0, false});
    auto hash = [](int64_t k) {
      k ^= k >> 33; k *= 0xff51afd7ed558ccdULL; k ^= k >> 33;
      return static_cast<size_t>(k);
    };
    for (const Vec2& p : pts) {
      const int64_t kx = static_cast<int64_t>(std::floor(p.x / voxel));
      const int64_t ky = static_cast<int64_t>(std::floor(p.y / voxel));
      const int64_t key = (kx << 32) ^ (ky & 0xFFFFFFFFLL);
      size_t i = hash(key) & (table.size() - 1);
      while (table[i].used && table[i].key != key)
        i = (i + 1) & (table.size() - 1);
      if (!table[i].used) {
        table[i] = {key, true};
        out.push_back(p);
      }
    }
    return out;
  }
};

// ------------------------------------------------------------------- matcher
struct MatchResult {
  Pose2D pose;
  double score = -1e9;  // mean log-odds under the points (comparable)
};

class ScanMatcher {
 public:
  struct Params {
    double searchXY = 0.15;        // prompt §5: +-0.15 m
    double searchYawDeg = 10.0;    //            +-10 deg
    double coarseStepXY = 0.05;    // m (on the 0.2 m strided score)
    double coarseStepYawDeg = 2.0;
    double fineStepXY = 0.025;
    double fineStepYawDeg = 1.0;
    int gnIters = 5;
    double gnStepXY = 0.01;
    double gnStepYawDeg = 0.5;
  };

  // Two ctors (not `= Params()` default arg): a nested class's default
  // member initializers can't be used in the enclosing class's default
  // arguments - the enclosing class is incomplete there.
  ScanMatcher() : p_() {}
  explicit ScanMatcher(const Params& p) : p_(p) {}

  // points: base-frame (pre-processed). guess: EKF/odometry seed.
  MatchResult match(const OccupancyGrid& grid,
                    const std::vector<Vec2>& points,
                    const Pose2D& guess) const {
    MatchResult best;
    best.pose = guess;
    if (points.empty()) return best;

    // Pass 1: coarse, 0.2 m strided scoring (smooth surface).
    best = search(grid, points, guess, p_.coarseStepXY, p_.coarseStepYawDeg,
                  /*strideCells=*/4);
    // Pass 2: fine on the true grid, around the coarse winner but still
    // inside the original +-window of the seed (prompt's search window).
    Pose2D fineSeed = best.pose;
    // Clamp the fine seed back into the seed window so a bad coarse vote
    // cannot wander off.
    const double dx = fineSeed.x - guess.x, dy = fineSeed.y - guess.y;
    const double d = std::hypot(dx, dy);
    if (d > p_.searchXY) {
      fineSeed.x = guess.x + dx / d * p_.searchXY;
      fineSeed.y = guess.y + dy / d * p_.searchXY;
    }
    best = search(grid, points, fineSeed, p_.fineStepXY, p_.fineStepYawDeg, 1);
    // Pass 3: Gauss-Newton on the score.
    best = gaussNewton(grid, points, best.pose);
    return best;
  }

  // Score of a pose: mean log-odds under the transformed points. strideCells
  // > 1 samples every strideCells-th cell (0.2 m view of a 0.05 m grid).
  static double score(const OccupancyGrid& grid,
                      const std::vector<Vec2>& points, const Pose2D& pose,
                      int strideCells = 1) {
    double sum = 0.0;
    int n = 0;
    for (const Vec2& p : points) {
      const Vec2 w = pose.translation() + rotate(p, pose.theta);
      int gx, gy;
      grid.worldToCell(w.x, w.y, gx, gy);
      if (strideCells > 1) {
        // Centre of the strided bucket containing (gx,gy).
        gx = (gx / strideCells) * strideCells + strideCells / 2;
        gy = (gy / strideCells) * strideCells + strideCells / 2;
      }
      sum += grid.logOdds(gx, gy);
      ++n;
    }
    return n ? sum / n : -1e9;
  }

 private:
  Params p_;

  MatchResult search(const OccupancyGrid& grid,
                     const std::vector<Vec2>& points, const Pose2D& seed,
                     double stepXY, double stepYawDeg, int stride) const {
    MatchResult best;
    best.pose = seed;
    best.score = score(grid, points, seed, stride);
    const int nx = static_cast<int>(std::ceil(p_.searchXY / stepXY));
    const int ny = nx;
    const int nth = static_cast<int>(std::ceil(p_.searchYawDeg / stepYawDeg));
    for (int ix = -nx; ix <= nx; ++ix) {
      for (int iy = -ny; iy <= ny; ++iy) {
        for (int it = -nth; it <= nth; ++it) {
          Pose2D cand(seed.x + ix * stepXY, seed.y + iy * stepXY,
                      seed.theta + it * stepYawDeg * kPi / 180.0);
          const double s = score(grid, points, cand, stride);
          if (s > best.score) {
            best.score = s;
            best.pose = cand;
          }
        }
      }
    }
    return best;
  }

  MatchResult gaussNewton(const OccupancyGrid& grid,
                          const std::vector<Vec2>& points,
                          const Pose2D& start) const {
    Pose2D pose = start;
    double sc = score(grid, points, pose);
    for (int iter = 0; iter < p_.gnIters; ++iter) {
      const double sx = p_.gnStepXY, st = p_.gnStepYawDeg * kPi / 180.0;
      const double s0 = sc;
      const double sxp = score(grid, points, {pose.x + sx, pose.y, pose.theta});
      const double sxm = score(grid, points, {pose.x - sx, pose.y, pose.theta});
      const double syp = score(grid, points, {pose.x, pose.y + sx, pose.theta});
      const double sym = score(grid, points, {pose.x, pose.y - sx, pose.theta});
      const double stp = score(grid, points, {pose.x, pose.y, pose.theta + st});
      const double stm = score(grid, points, {pose.x, pose.y, pose.theta - st});
      const double gx = (sxp - sxm) / (2 * sx);
      const double gy = (syp - sym) / (2 * sx);
      const double gt = (stp - stm) / (2 * st);
      // Gradient ascent on the score with a damped step (the score surface
      // is not quadratic, so full GN steps overshoot; climb instead).
      const double norm = std::sqrt(gx * gx + gy * gy + gt * gt * 0.01) + 1e-12;
      const double alpha = 0.6;
      Pose2D cand(pose.x + alpha * gx / norm * sx,
                  pose.y + alpha * gy / norm * sx,
                  pose.theta + alpha * gt / norm * st);
      const double s1 = score(grid, points, cand);
      if (s1 <= s0) break;   // no improvement: done
      pose = cand;
      sc = s1;
    }
    return {pose, sc};
  }
};

}  // namespace vacuum
