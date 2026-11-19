// mapper.h - occupancy-grid SLAM (prompt §5): scan matching against the
// growing map, keyframes, a pose graph with loop closure, and re-render
// after optimization.
//
// Data flow per scan:
//   1. correlative match against the master grid, seeded by the EKF pose
//      (scan_matcher.h) -> refined pose
//   2. if the pose moved > keyframe thresholds vs the last keyframe, store
//      a keyframe and integrate the scan into the master grid
//   3. every ~submapScans keyframes, look for loop closures: older
//      keyframes whose (current) pose is within loopSeparationM get their
//      relative pose re-measured by matching; score above loopScoreTh adds
//      a loop edge
//   4. on a new loop edge, optimise the pose graph (Eigen, Levenberg-
//      Marquardt, Huber weights) over keyframe poses, then RE-RENDER the
//      master grid from all keyframes at their optimised poses
//
// Why re-render instead of incremental correction: log-odds cells cannot be
// "un-integrated" after a loop jump; re-rendering from keyframe point
// clouds is exact and cheap at our keyframe counts (<100).

#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Dense>

#include "vacuum/math.h"
#include "vacuum/occupancy_grid.h"
#include "vacuum/scan_matcher.h"

namespace vacuum {

class Mapper {
 public:
  struct Params {
    ScanMatcher::Params matcher;
    double keyframeTransM = 0.2;      // prompt §5
    double keyframeRotDeg = 10.0;
    int scansPerSubmap = 60;          // loop-check cadence
    double loopSeparationM = 4.0;     // prompt: older submaps within 4 m
    double loopScoreTh = 0.5;         // mean log-odds to accept a loop
    int minKeyframeGapForLoop = 8;    // never close against a neighbour
    double huberDelta = 0.05;         // m on translation residuals
    int lmIters = 12;
  };

  struct Keyframe {
    Pose2D pose;                    // optimised map-frame pose
    std::vector<Vec2> pts;          // base-frame pre-processed points
  };

  struct Edge {
    int i = 0, j = 0;               // keyframe indices (i -> j)
    Pose2D relative;                // measured pose of j in frame i
    double weightT = 1.0;           // 1/sigma^2 on translation
    double weightR = 1.0;           // 1/sigma^2 on rotation
    bool loop = false;
  };

  // Two ctors: nested Params' default member initializers can't be used in
  // the enclosing class's default arguments (same rule as ScanMatcher/Ekf).
  Mapper() : p_(), matcher_(p_.matcher) {}
  explicit Mapper(const Params& p) : p_(p), matcher_(p.matcher) {}

  // Feed one pre-processed scan with the EKF pose guess (map frame).
  // Returns the matched pose (map frame).
  Pose2D process(const Pose2D& guess, const std::vector<Vec2>& points) {
    Pose2D pose = guess;
    double matchScore = -1e9;
    if (!keyframes_.empty() && !points.empty()) {
      const MatchResult m = matcher_.match(grid_, points, guess);
      pose = m.pose;
      matchScore = m.score;
    }

    if (keyframes_.empty()) {
      addKeyframe(pose, points);
      integrate(pose, points);
      return pose;
    }

    const Keyframe& last = keyframes_.back();
    const double trans = std::hypot(pose.x - last.pose.x, pose.y - last.pose.y);
    const double rot = std::abs(normalizeAngle(pose.theta - last.pose.theta));
    if (trans > p_.keyframeTransM ||
        rot > p_.keyframeRotDeg * kPi / 180.0) {
      // Sequential edge: relative pose from the MATCHED poses (the match
      // already rejected odometry slip against the map).
      Edge e;
      e.i = static_cast<int>(keyframes_.size()) - 1;
      e.j = static_cast<int>(keyframes_.size());
      e.relative = Pose2D::relative(last.pose, pose);
      edges_.push_back(e);
      addKeyframe(pose, points);
      integrate(pose, points);
      ++scansSinceLoopCheck_;

      if (scansSinceLoopCheck_ >= p_.scansPerSubmap) {
        scansSinceLoopCheck_ = 0;
        detectLoops();
      }
    }
    (void)matchScore;
    return pose;
  }

  const OccupancyGrid& map() const { return grid_; }
  const std::vector<Keyframe>& keyframes() const { return keyframes_; }
  const std::vector<Edge>& edges() const { return edges_; }
  int loopClosures() const {
    int n = 0;
    for (const Edge& e : edges_) n += e.loop ? 1 : 0;
    return n;
  }
  // True once at least one loop edge has been found and optimised.
  bool loopClosed() const { return loopClosures() > 0; }

  // ---- persistence hooks (Phase 3 MapStore wires these to files) --------
  std::string serializeGrid() const { return grid_.serializeRle(); }

 private:
  Params p_;
  ScanMatcher matcher_;
  OccupancyGrid grid_{0.05};
  std::vector<Keyframe> keyframes_;
  std::vector<Edge> edges_;
  int scansSinceLoopCheck_ = 0;

  void addKeyframe(const Pose2D& pose, const std::vector<Vec2>& points) {
    keyframes_.push_back(Keyframe{pose, points});
  }

  // Integrate one scan: each hit point clears its ray and marks its cell.
  void integrate(const Pose2D& pose, const std::vector<Vec2>& points) {
    const Vec2 origin = pose.translation();
    for (const Vec2& p : points) {
      const Vec2 w = origin + rotate(p, pose.theta);
      grid_.rayHit(origin, w);
    }
  }

  // Loop detection: match the LAST keyframe's scan against the master grid
  // (which contains the old area) seeded at the current pose; a strong
  // score near an old keyframe = the map saw itself again.
  void detectLoops() {
    if (keyframes_.size() < static_cast<size_t>(p_.minKeyframeGapForLoop) + 2)
      return;
    const Keyframe& cur = keyframes_.back();
    // Candidate: an older keyframe whose pose is within separation and far
    // enough in index. Seed the matcher at the current pose (odometry said
    // we are here; if the map disagrees strongly but scores well, the
    // disagreement IS the loop correction).
    for (size_t oi = 0; oi + p_.minKeyframeGapForLoop < keyframes_.size() - 1;
         ++oi) {
      const Keyframe& old = keyframes_[oi];
      const double d = std::hypot(old.pose.x - cur.pose.x,
                                  old.pose.y - cur.pose.y);
      if (d > p_.loopSeparationM) continue;

      const MatchResult m = matcher_.match(grid_, cur.pts, cur.pose);
      if (m.score < p_.loopScoreTh) continue;

      // Loop edge between the old keyframe and the current one: relative
      // pose implied by the match (current scan matched into the map that
      // old keyframe helped build).
      Edge e;
      e.i = static_cast<int>(oi);
      e.j = static_cast<int>(keyframes_.size()) - 1;
      e.relative = Pose2D::relative(old.pose, m.pose);
      e.loop = true;
      edges_.push_back(e);
      optimize();
      rerender();
      return;   // one loop per check keeps the graph stable
    }
  }

  // ---- pose graph: LM over keyframe poses with Huber weights -------------
  // Residual for edge (i,j): r = (relative(i)^-1 * j) - measured,3-vector
  // (dx, dy, dtheta). Normal equations solved densely (N < ~200 poses ->
  // 600x600, milliseconds on a phone).
  void optimize() {
    const int n = static_cast<int>(keyframes_.size());
    if (n < 2 || edges_.empty()) return;

    for (int iter = 0; iter < p_.lmIters; ++iter) {
      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3 * n, 3 * n);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(3 * n);
      double cost = 0.0;

      for (const Edge& e : edges_) {
        const Pose2D& pi = keyframes_[e.i].pose;
        const Pose2D& pj = keyframes_[e.j].pose;
        // predicted relative
        const Pose2D pred = Pose2D::relative(pi, pj);
        Eigen::Vector3d r(pred.x - e.relative.x, pred.y - e.relative.y,
                          normalizeAngle(pred.theta - e.relative.theta));

        // Huber weight on the translation magnitude (prompt §5).
        const double rn = r.head<2>().norm();
        double w = 1.0;
        if (rn > p_.huberDelta) w = p_.huberDelta / rn;
        const double wt = w * e.weightT;
        const double wr = w * e.weightR;
        cost += wt * r.head<2>().squaredNorm() + wr * r[2] * r[2];

        // Jacobians (SE2, see header): d r / d xi, d r / d xj.
        const double c = std::cos(pi.theta), s = std::sin(pi.theta);
        const double dx = pj.x - pi.x, dy = pj.y - pi.y;
        // d(pred_t)/d ti = -R^T ; d(pred_t)/d tj = R^T
        // d(pred_t)/d theta_i = dR^T/dth * (tj-ti)
        Eigen::Matrix2d Rt;
        Rt << c, s, -s, c;
        Eigen::Matrix2d dRtdth;
        dRtdth << -s, c, -c, -s;
        Eigen::Matrix<double, 3, 3> Ji, Jj;
        Ji.setZero();
        Jj.setZero();
        Ji.block<2, 2>(0, 0) = -Rt;
        Ji.block<2, 1>(0, 2) = dRtdth * Eigen::Vector2d(dx, dy);
        Ji(2, 2) = -1.0;
        Jj.block<2, 2>(0, 0) = Rt;
        Jj(2, 2) = 1.0;

        Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
        W(0, 0) = W(1, 1) = wt;
        W(2, 2) = wr;

        const int ii = 3 * e.i, jj = 3 * e.j;
        H.block<3, 3>(ii, ii) += Ji.transpose() * W * Ji;
        H.block<3, 3>(ii, jj) += Ji.transpose() * W * Jj;
        H.block<3, 3>(jj, ii) += Jj.transpose() * W * Ji;
        H.block<3, 3>(jj, jj) += Jj.transpose() * W * Jj;
        b.segment<3>(ii) += Ji.transpose() * W * r;
        b.segment<3>(jj) += Jj.transpose() * W * r;
      }

      // Fix gauge: pin the first keyframe (it defines the map frame).
      H.block<3, 3>(0, 0) += Eigen::Matrix3d::Identity() * 1e6;
      b.segment<3>(0).setZero();

      // Levenberg damping.
      H.diagonal().array() += 1e-6;
      const Eigen::VectorXd delta = H.ldlt().solve(-b);
      if (!delta.allFinite()) break;

      double maxStep = 0.0;
      for (int k = 0; k < n; ++k) {
        keyframes_[k].pose.x += delta(3 * k);
        keyframes_[k].pose.y += delta(3 * k + 1);
        keyframes_[k].pose.theta =
            normalizeAngle(keyframes_[k].pose.theta + delta(3 * k + 2));
        maxStep = std::max(maxStep, std::abs(delta(3 * k)) +
                                        std::abs(delta(3 * k + 1)) +
                                        std::abs(delta(3 * k + 2)));
      }
      if (maxStep < 1e-6) break;
      (void)cost;
    }
  }

  // Re-render the master grid from every keyframe at its optimised pose.
  void rerender() {
    grid_.clear();
    for (const Keyframe& kf : keyframes_) integrate(kf.pose, kf.pts);
  }
};

}  // namespace vacuum
