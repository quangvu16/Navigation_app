// amcl.h - AMCL-style Monte Carlo localization on a saved map (prompt §5).
//
// - Likelihood field: per-cell distance to the nearest occupied cell
//   (two-pass chamfer distance transform), precomputed once per map. Scan
//   score = product of field values at the beam endpoints (a beam that
//   lands near a wall scores high).
// - KLD-adaptive particle count300..3000 (prompt): after each resample,
//   the number of particles follows the KLD bound on the particle
//   histogram's occupied bins (epsilon = 0.1, confidence 0.99, Wilson-
//   Hilferty chi-square approximation).
// - Augmented MCL (SThrun): injected random particles when the filter
//   senses it has lost the robot (weight collapse) = kidnapping recovery;
//   global init spreads particles over free space.
//
// Motion model: odometry-based sampling with alphas (Nav2's differential
// motion model hints: alphas0.05). The caller feeds odom deltas between
// updates (tick-derived v/ω, or pose deltas).

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "vacuum/math.h"
#include "vacuum/occupancy_grid.h"

namespace vacuum {

class LikelihoodField {
 public:
  // Build from a map: distance transform of the occupied cells.
  void build(const OccupancyGrid& grid) {
    gx0_ = grid.originX();
    gy0_ = grid.originY();
    w_ = grid.width();
    h_ = grid.height();
    res_ = grid.resolution();
    dist_.assign(static_cast<size_t>(w_) * h_, 1e9f);
    for (int y = 0; y < h_; ++y)
      for (int x = 0; x < w_; ++x) {
        if (grid.occupied(gx0_ + x, gy0_ + y))
          dist_[static_cast<size_t>(y) * w_ + x] = 0.0f;
      }
    // Two-pass chamfer (3-4 weights scaled by res): forward then backward.
    const float a = 1.0f, b = 1.41421356f;   // in cell units
    for (int y = 0; y < h_; ++y)
      for (int x = 0; x < w_; ++x) {
        float& d = dist_[static_cast<size_t>(y) * w_ + x];
        if (x > 0) d = std::min(d, dist_[static_cast<size_t>(y) * w_ + x - 1] + a);
        if (y > 0) d = std::min(d, dist_[static_cast<size_t>(y - 1) * w_ + x] + a);
        if (x > 0 && y > 0)
          d = std::min(d, dist_[static_cast<size_t>(y - 1) * w_ + x - 1] + b);
        if (x + 1 < w_ && y > 0)
          d = std::min(d, dist_[static_cast<size_t>(y - 1) * w_ + x + 1] + b);
      }
    for (int y = h_ - 1; y >= 0; --y)
      for (int x = w_ - 1; x >= 0; --x) {
        float& d = dist_[static_cast<size_t>(y) * w_ + x];
        if (x + 1 < w_) d = std::min(d, dist_[static_cast<size_t>(y) * w_ + x + 1] + a);
        if (y + 1 < h_) d = std::min(d, dist_[static_cast<size_t>(y + 1) * w_ + x] + a);
        if (x + 1 < w_ && y + 1 < h_)
          d = std::min(d, dist_[static_cast<size_t>(y + 1) * w_ + x + 1] + b);
        if (x > 0 && y + 1 < h_)
          d = std::min(d, dist_[static_cast<size_t>(y + 1) * w_ + x - 1] + b);
      }
    for (float& d : dist_) d *= static_cast<float>(res_);
  }

  // Distance (m) from a world point to the nearest occupied cell.
  double distance(double wx, double wy) const {
    const int x = static_cast<int>(std::floor(wx / res_)) - gx0_;
    const int y = static_cast<int>(std::floor(wy / res_)) - gy0_;
    if (x < 0 || y < 0 || x >= w_ || y >= h_) return 1e9;
    return dist_[static_cast<size_t>(y) * w_ + x];
  }

  // Beam likelihood: gaussian around the field distance (z_hit model with
  // a small z_rand floor so an outlying beam cannot zero the product).
  double beamWeight(double beamDist, double sigma = 0.10) const {
    const double p = std::exp(-(beamDist * beamDist) / (2.0 * sigma * sigma));
    return 0.95 * p + 0.05 * (1.0 / (std::sqrt(2 * kPi) * sigma * 10.0));
  }

 private:
  int gx0_ = 0, gy0_ = 0, w_ = 0, h_ = 0;
  double res_ = 0.05;
  std::vector<float> dist_;
};

class Mcl {
 public:
  struct Params {
    int minParticles = 300;      // prompt: KLD adaptive 300..3000
    int maxParticles = 3000;
    double kldEpsilon = 0.1;     // KLD bound on histogram error
    double resampleThreshold = 0.5;  // nEff < this * N -> resample
    // Odometry motion model alphas. Kept SMALL (0.01/0.02, tighter than
    // Nav2's generic0.05): with larger noise the particle cloud smears
    // faster than the likelihood can correct during a turn, a scan-aliased
    // injected particle briefly outscores the true cluster, and resampling
    // then erases the truth forever (observed in the walk test).
    double alpha1 = 0.01;   // rot noise from rot
    double alpha2 = 0.01;   // rot noise from trans
    double alpha3 = 0.02;   // trans noise from trans
    double alpha4 = 0.02;   // trans noise from rot
    double initXYStd = 0.10;    // global init spread per cell-ish
    double initThetaStd = 0.35; // ~20 deg
    // Mean beam WEIGHT (not log) below which the BEST particle counts as
    // lost (looksLost(); caller re-inits globally). Measured margins: a
    // converged filter scores ~0.93, a kidnapped cloud scoring against the
    // wrong room ~0.11 - so 0.15 separates them with room to spare.
    double kidnapMeanLogW = 0.15;
    double laserSigma = 0.10;   // likelihood field sigma
    double injectFraction = 0.02;  // augmented-MCL random particles/resample
    // Softmax temperature annealing: early updates divide log-weights by
    // temp0 (soft), decaying to 1 with time constant tempTau updates. A
    // plain softmax on log-likelihoods is winner-take-all: the single
    // best-scoring hypothesis (often a scan alias) takes the whole
    // population on update #1 and the truth never gets another chance.
    // Soft early weights keep several modes alive until odometry motion
    // discriminates them.
    double temp0 = 20.0;
    double tempTau = 50.0;
    int updateEvery = 1;        // updates between resamples
  };

  // Two ctors (nested-Params NSDMI rule; same as ScanMatcher/Ekf/Mapper).
  Mcl() : p_() { reset(); }
  explicit Mcl(const Params& p) : p_(p) { reset(); }

  // Fresh filter: no particles until initGlobal/initAround.
  void reset() {
    particles_.clear();
    weights_.clear();
  }

  bool initialized() const { return !particles_.empty(); }

  // Global localization (prompt §5: spreading particles over free space).
  // Two layers: a DETERMINISTIC coarse pose grid (every ~0.5 m and 45 deg
  // over free cells) so a hypothesis within scan-matching distance of the
  // truth always exists, then random fill up to the full budget. Purely
  // random init at minParticles often has no good-position-good-heading
  // particle at all, and the first mediocre softmax winner collapses the
  // whole population onto the wrong pose.
  void initGlobal(const OccupancyGrid& map, std::mt19937& rng) {
    std::uniform_real_distribution<double> u(0.0, 1.0);
    particles_.clear();
    weights_.clear();
    bestLogW_ = -1e300;

    const double step = 0.25;   // finer than the map cell pattern so the
    // weighted mean can sit within ~0.15 m of the truth after updates
    const double x0 = map.originX() * map.resolution();
    const double y0 = map.originY() * map.resolution();
    const double x1 = x0 + map.width() * map.resolution();
    const double y1 = y0 + map.height() * map.resolution();
    for (double x = x0 + step * 0.5; x < x1; x += step)
      for (double y = y0 + step * 0.5; y < y1; y += step) {
        int gx, gy;
        map.worldToCell(x, y, gx, gy);
        if (!map.free(gx, gy)) continue;
        for (int t = 0; t < 8; ++t) {
          particles_.push_back({x, y, -kPi + t * (kPi / 4.0)});
        }
      }

    // Fill to the budget with uniform random free poses.
    const int n = p_.maxParticles;
    while (static_cast<int>(particles_.size()) < n) {
      const Pose2D rp = randomFreePose(map, rng);
      if (rp.x == 1e300) break;
      particles_.push_back({rp.x, rp.y, rp.theta});
    }
    const double w = 1.0 / std::max<size_t>(1, particles_.size());
    weights_.assign(particles_.size(), w);
  }

  // Init around a known pose ("set pose" or odom seed).
  void initAround(const Pose2D& pose, std::mt19937& rng) {
    std::normal_distribution<double> nxy(0.0, p_.initXYStd);
    std::normal_distribution<double> nth(0.0, p_.initThetaStd);
    particles_.clear();
    weights_.clear();
    const int n = p_.minParticles;
    for (int i = 0; i < n; ++i) {
      particles_.push_back({pose.x + nxy(rng), pose.y + nxy(rng),
                            normalizeAngle(pose.theta + nth(rng))});
      weights_.push_back(1.0 / n);
    }
  }

  // Motion update: odom delta (relative motion) sampled with the alphas
  // (odometry-based sampling, Thrun et al.):
  //   dth' = dth - sqrt(a1*dth^2 + a2*dr^2) * n1
  //   dr'  = dr  - sqrt(a3*dr^2 + a4*dth^2) * n2
  void predict(const Pose2D& odomDelta, std::mt19937& rng) {
    if (particles_.empty()) return;
    const double dr = std::hypot(odomDelta.x, odomDelta.y);
    const double dth = odomDelta.theta;
    std::normal_distribution<double> n(0.0, 1.0);
    for (auto& p : particles_) {
      const double dth_p = dth - std::sqrt(
          p_.alpha1 * dth * dth + p_.alpha2 * dr * dr) * n(rng);
      const double dr_p = dr - std::sqrt(
          p_.alpha3 * dr * dr + p_.alpha4 * dth * dth) * n(rng);
      const double thMid = p.theta + 0.5 * dth_p;
      p.x += dr_p * std::cos(thMid);
      p.y += dr_p * std::sin(thMid);
      p.theta = normalizeAngle(p.theta + dth_p);
    }
  }

  // Measurement update against the likelihood field. `points` are base-
  // frame scan points (pre-processed); each particle transforms them into
  // the map. Weights are accumulated in the log domain (90 beams x log(p)
  // underflows doubles in the linear domain), then softmaxed against the
  // best log-weight.
  //
  // `mapForInject` (optional): the map used by augmented-MCL's random
  // particle injection at resample time - the cure for premature collapse
  // AND the kidnap case (prompt §5).
  void update(const LikelihoodField& field,
              const std::vector<Vec2>& points,
              double laserSigma,
              const OccupancyGrid* mapForInject = nullptr) {
    if (particles_.empty() || points.empty()) return;
    std::vector<double> logW(particles_.size(), -1e300);
    double maxLog = -1e300;
    lastBeamCount_ = static_cast<int>((points.size() + 3) / 4);
    for (size_t i = 0; i < particles_.size(); ++i) {
      const Particle& p = particles_[i];
      double lw = 0.0;
      // Subsample beams for speed (~90 of360).
      for (size_t k = 0; k < points.size(); k += 4) {
        const Vec2 w = Vec2(p.x, p.y) + rotate(points[k], p.theta);
        const double d = field.distance(w.x, w.y);
        lw += std::log(d > 4.0 ? 1e-6 : field.beamWeight(d, laserSigma));
      }
      logW[i] = lw;
      if (lw > maxLog) maxLog = lw;
    }
    bestLogW_ = maxLog;
    // Tempered softmax (see Params::temp0): soft while the filter is young
    // so scan aliases and the true pose coexist; hard once settled.
    const double temp = 1.0 + (p_.temp0 - 1.0) *
        std::exp(-static_cast<double>(updates_) / p_.tempTau);
    double sum = 0.0;
    for (size_t i = 0; i < logW.size(); ++i) {
      weights_[i] = std::exp((logW[i] - maxLog) / temp);
      sum += weights_[i];
    }
    if (sum <= 0.0) {
      const double w = 1.0 / weights_.size();
      std::fill(weights_.begin(), weights_.end(), w);
      return;
    }
    for (double& w : weights_) w /= sum;
    avgWeight_ = sum / static_cast<double>(weights_.size());
    ++updates_;
    if (updates_ % p_.updateEvery == 0)
      resample(rng_, field, points, laserSigma, mapForInject);
  }

  // Estimate = the HIGHEST-WEIGHT particle (the mode), not the weighted
  // mean: with two live hypotheses (true pose + scan alias) the mean sits
  // somewhere between the rooms, which is useless for control.
  Pose2D pose() const {
    if (particles_.empty()) return Pose2D();
    size_t best = 0;
    for (size_t i = 1; i < weights_.size(); ++i)
      if (weights_[i] > weights_[best]) best = i;
    return Pose2D(particles_[best].x, particles_[best].y,
                  particles_[best].theta);
  }

  double averageWeight() const { return avgWeight_; }
  int particleCount() const { return static_cast<int>(particles_.size()); }

  // Kidnap/lost detector: the BEST particle's mean beam likelihood. When
  // every particle scores below kidnapMeanLogW, no hypothesis matches the
  // map - the caller re-inits globally (augmented-MCL recovery, prompt §5).
  bool looksLost() const {
    if (!initialized() || bestLogW_ <= -1e299) return false;
    const int beams = std::max(1, static_cast<int>(lastBeamCount_));
    return bestLogW_ / beams < std::log(p_.kidnapMeanLogW);
  }

  // Expose for tests/diagnostics.
  struct Particle {
    double x, y, theta;
  };
  const std::vector<Particle>& particles() const { return particles_; }

 private:

  Params p_;
  std::vector<Particle> particles_;
  std::vector<double> weights_;
  double avgWeight_ = 0.0;
  double bestLogW_ = -1e300;
  int lastBeamCount_ = 0;
  int updates_ = 0;
  std::mt19937 rng_{42};

  void resample(std::mt19937& rng, const LikelihoodField& field,
                const std::vector<Vec2>& points, double sigma,
                const OccupancyGrid* map) {
    if (particles_.empty()) return;
    // Effective sample size.
    double w2 = 0;
    for (double w : weights_) w2 += w * w;
    const double nEff = 1.0 / std::max(w2, 1e-12);
    if (nEff > p_.resampleThreshold * static_cast<double>(particles_.size()))
      return;   // still healthy

    // Systematic resampling.
    std::vector<Particle> next;
    next.reserve(particles_.size());
    std::uniform_real_distribution<double> u(0.0,
        1.0 / static_cast<double>(particles_.size()));
    double r = u(rng);
    double c = weights_[0];
    size_t i = 0;
    for (size_t j = 0; j < particles_.size(); ++j) {
      const double uj = r + static_cast<double>(j) / particles_.size();
      while (uj > c && i + 1 < weights_.size()) { ++i; c += weights_[i]; }
      next.push_back(particles_[i]);
    }
    particles_ = std::move(next);

    // KLD-adaptive count: histogram the particles into bins (0.05 m /
    // ~3 deg), then take the KLD bound as the next count.
    const double binXY = 0.05, binTh = 3.0 * kPi / 180.0;
    std::vector<uint64_t> keys;
    keys.reserve(particles_.size());
    for (const auto& p : particles_) {
      const int64_t bx = static_cast<int64_t>(std::floor(p.x / binXY));
      const int64_t by = static_cast<int64_t>(std::floor(p.y / binXY));
      const int64_t bt = static_cast<int64_t>(std::floor(
          normalizeAngle(p.theta) / binTh + 1024.0));
      keys.push_back((static_cast<uint64_t>(bx & 0xFFFF) << 48) |
                     (static_cast<uint64_t>(by & 0xFFFF) << 32) |
                     static_cast<uint64_t>(bt & 0xFFFFFFFF));
    }
    std::sort(keys.begin(), keys.end());
    const size_t k = static_cast<size_t>(
        std::unique(keys.begin(), keys.end()) - keys.begin());

    int n = static_cast<int>(particles_.size());
    if (k >= 2) {
      // KLD: N = (k-1)/(2 eps) * (1 - 2/(9(k-1)) + z * sqrt(2/(9(k-1))))^3
      // with z = z_{0.99} ~ 2.33 (confidence0.99), eps = kldEpsilon.
      const double km1 = static_cast<double>(k - 1);
      const double z = 2.326347874;   // Phi^-1(0.99)
      const double core = 1.0 - 2.0 / (9.0 * km1) +
          z * std::sqrt(2.0 / (9.0 * km1));
      const double nKld = (km1 / (2.0 * p_.kldEpsilon)) * core * core * core;
      n = static_cast<int>(std::ceil(nKld));
      n = std::max(p_.minParticles, std::min(p_.maxParticles, n));
    } else {
      n = p_.maxParticles;   // very uncertain: use the full budget
    }

    // Grow/shrink to the KLD count (noise on new particles).
    std::normal_distribution<double> nxy(0.0, binXY);
    std::normal_distribution<double> nth(0.0, binTh);
    while (static_cast<int>(particles_.size()) < n) {
      const Particle& src = particles_[rng() % particles_.size()];
      particles_.push_back({src.x + nxy(rng), src.y + nxy(rng),
                            normalizeAngle(src.theta + nth(rng))});
    }
    if (static_cast<int>(particles_.size()) > n)
      particles_.resize(static_cast<size_t>(n));

    // Augmented MCL (prompt §5): inject random particles over free space -
    // AFTER the KLD shrink (injecting before it gets truncated away by
    // resize). Without this the first softmax winner - even a mediocre one -
    // takes the whole population and the filter can never recover the truth
    // (premature collapse) or a kidnapped robot.
    if (map != nullptr) {
      const int inj = std::max(1, static_cast<int>(
          p_.injectFraction * static_cast<double>(particles_.size())));
      for (int a = 0; a < inj; ++a) {
        const Pose2D rp = randomFreePose(*map, rng);
        if (rp.x == 1e300) break;
        particles_.push_back({rp.x, rp.y, rp.theta});
      }
    }

    const double w = 1.0 / particles_.size();
    weights_.assign(particles_.size(), w);
    avgWeight_ = w;
    (void)field;
    (void)points;
    (void)sigma;
  }

  // Uniform pose over the map's free cells (rejection sampling: interiors
  // are mostly free, so a handful of tries always lands).
  static Pose2D randomFreePose(const OccupancyGrid& map, std::mt19937& rng) {
    std::uniform_real_distribution<double> u(0.0, 1.0);
    for (int tries = 0; tries < 200; ++tries) {
      const int x = static_cast<int>(u(rng) * map.width());
      const int y = static_cast<int>(u(rng) * map.height());
      const int gx = map.originX() + std::min(map.width() - 1, std::max(0, x));
      const int gy = map.originY() + std::min(map.height() - 1, std::max(0, y));
      if (map.free(gx, gy)) {
        const Vec2 c = map.cellCentre(gx, gy);
        return Pose2D(c.x, c.y, u(rng) * 2.0 * kPi - kPi);
      }
    }
    return Pose2D(1e300, 1e300, 0.0);   // failed
  }
};

}  // namespace vacuum
