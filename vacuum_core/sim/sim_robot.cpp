// sim_robot.cpp — see sim_robot.h for the model's intent and conventions.

#include "sim_robot.h"

#include <algorithm>
#include <cmath>

namespace vacuum {

namespace {
constexpr double kPi_ = 3.14159265358979323846;
// HC-SR04 max useful echo (firmware SONAR_ECHO_TIMEOUT_US ~ 4.3 m).
constexpr double kSonarMaxM = 4.3;
constexpr double kSonarPeriodS = 0.12;  // ~8 Hz per sensor (firmware: 120 ms)
}  // namespace

SimRobot::SimRobot(const RobotConfig& cfg, SimWorld world,
                   const SimNoise& noise, uint32_t seed)
    : cfg_(cfg), world_(std::move(world)), noise_(noise), rng_(seed) {
  s_.ranges.assign(static_cast<size_t>(cfg_.scanBins), 0.0f);
  s_.intensity.assign(static_cast<size_t>(cfg_.scanBins), 0);
  s_.rotationHz = 6.0;
  if (noise_.enabled) {
    gyroBias_ = gaussian(noise_.gyroBiasStd);
  }
}

void SimRobot::setWheelRpm(double rpmL, double rpmR) {
  cmdRpmL_ = rpmL;
  cmdRpmR_ = rpmR;
}

void SimRobot::setPose(const Pose2D& p) {
  s_.truePose = p;
  s_.odomPose = p;
}

double SimRobot::gaussian(double stddev) {
  if (!noise_.enabled || stddev <= 0.0) return 0.0;
  return normal_(rng_) * stddev;
}

void SimRobot::step(double dt) {
  if (dt <= 0.0) return;
  s_.scanFresh = false;  // "fresh" = a revolution completed during THIS step
  timeS_ += dt;

  // --- wheel dynamics: first-order lag toward the commanded RPM -----------
  const double targetL = cmdRpmL_ * 2.0 * kPi_ / 60.0;  // rad/s
  const double targetR = cmdRpmR_ * 2.0 * kPi_ / 60.0;
  const double alpha =
      noise_.enabled ? 1.0 - std::exp(-dt / noise_.motorTauS) : 1.0;
  wheelRadPerSL_ += (targetL - wheelRadPerSL_) * alpha;
  wheelRadPerSR_ += (targetR - wheelRadPerSR_) * alpha;

  const double wL = wheelRadPerSL_ + gaussian(noise_.wheelSpeedNoiseStd);
  const double wR = wheelRadPerSR_ + gaussian(noise_.wheelSpeedNoiseStd);

  // --- ground slip: a slowly drifting fraction of wheel speed lost, plus a
  // little uncommanded yaw (carpet / one wheel slipping more than the other).
  slipFrac_ += gaussian(noise_.slipWalkStd * std::sqrt(dt));
  slipFrac_ = std::clamp(slipFrac_, -0.1, 0.1);
  const double slip = noise_.enabled ? slipFrac_ : 0.0;

  const double r = cfg_.wheelRadiusM;
  const double v = (wL + wR) * 0.5 * r * (1.0 - slip);
  const double w = (wR - wL) * r / cfg_.trackWidthM +
                   gaussian(0.02 * std::abs(slip) / 0.015);

  // --- true pose: midpoint integration (exact for arcs at small dt) ------
  const double thMid = s_.truePose.theta + w * dt * 0.5;
  s_.truePose.x += v * std::cos(thMid) * dt;
  s_.truePose.y += v * std::sin(thMid) * dt;
  s_.truePose.theta = normalizeAngle(s_.truePose.theta + w * dt);
  s_.trueOmega = w;
  s_.gyroZ = w + gyroBias_ + gaussian(noise_.gyroNoiseStd);

  // --- odometry: what the hall ticks report (wheel rotation, not ground) --
  static thread_local double tickCarryL = 0.0, tickCarryR = 0.0;
  const double ticksPerRad = cfg_.ticksPerRev / (2.0 * kPi_);
  tickCarryL += wL * dt * ticksPerRad;
  tickCarryR += wR * dt * ticksPerRad;
  const int32_t dL = static_cast<int32_t>(tickCarryL);
  const int32_t dR = static_cast<int32_t>(tickCarryR);
  tickCarryL -= dL;
  tickCarryR -= dR;
  s_.ticksL += dL;
  s_.ticksR += dR;

  s_.rpmL = wL * 60.0 / (2.0 * kPi_) + gaussian(noise_.wheelSpeedNoiseStd * 2.0);
  s_.rpmR = wR * 60.0 / (2.0 * kPi_) + gaussian(noise_.wheelSpeedNoiseStd * 2.0);

  // Odom pose integrates the *measured* RPM (what HBT reports), exactly like
  // the app's odometry module will, so slip shows up as odom drift against
  // the true pose.
  const double vOd = (s_.rpmL + s_.rpmR) * 0.5 * (2.0 * kPi_ / 60.0) * r;
  const double wOd = (s_.rpmR - s_.rpmL) * (2.0 * kPi_ / 60.0) * r / cfg_.trackWidthM;
  const double thMidO = s_.odomPose.theta + wOd * dt * 0.5;
  s_.odomPose.x += vOd * std::cos(thMidO) * dt;
  s_.odomPose.y += vOd * std::sin(thMidO) * dt;
  s_.odomPose.theta = normalizeAngle(s_.odomPose.theta + wOd * dt);

  // --- battery + post-firmware output RPM (battery.h rules) --------------
  s_.batteryV = std::max(30.0, 39.0 - 0.0002 * timeS_);
  double scale = 1.0;
  if (s_.batteryV < cfg_.batteryCriticalV) scale = 0.0;
  else if (s_.batteryV < cfg_.batteryLowV) scale = 0.5;
  s_.outRpmL = cmdRpmL_ * scale;
  s_.outRpmR = cmdRpmR_ * scale;

  // --- sensors -----------------------------------------------------------
  acquireBeams(dt);
  sampleSonar();
}

void SimRobot::acquireBeams(double dt) {
  const int bins = cfg_.scanBins;
  scanRev_ += dt * s_.rotationHz;
  const double mountYaw = deg2rad(cfg_.lidarMountYawDeg);
  const double step = 2.0 * kPi_ / bins;
  const int dir = cfg_.lidarMirror ? -1 : +1;  // wire convention: clockwise

  // Laser origin: base frame offset, rotated into the world.
  const Vec2 origin =
      s_.truePose.translation() +
      rotate(Vec2(cfg_.lidarOffsetXM, cfg_.lidarOffsetYM), s_.truePose.theta);

  while (static_cast<int>(scanRev_ * bins) > beamsDone_) {
    const int i = beamsDone_;
    // Beam i: acquired when the head sweeps past bin i's angle. World angle
    // = zero mark (base theta + mount yaw) turned `dir * i * step`.
    const double ang = s_.truePose.theta + mountYaw + dir * i * step;
    uint8_t inten = 0;
    double range = world_.raycast(origin, ang, cfg_.lidarMaxRangeM, &inten);
    if (range >= cfg_.lidarMaxRangeM) {
      range = 0.0;  // wire convention: no return
      inten = 0;
    } else {
      range += gaussian(noise_.rangeNoiseStd);
      if (range < cfg_.lidarMinRangeM * 0.5) range = 0.0;
    }
    s_.ranges[static_cast<size_t>(i)] = static_cast<float>(range);
    s_.intensity[static_cast<size_t>(i)] = inten;
    ++beamsDone_;

    if (beamsDone_ >= bins) {
      // Revolution complete: publish the scan.
      s_.scanId++;
      s_.scanFresh = true;
      s_.scanStartS = scanStartS_;
      s_.scanEndS = timeS_;
      scanStartS_ = timeS_;
      beamsDone_ = 0;
      scanRev_ -= 1.0;  // keep the fractional phase: 6 Hz must stay exact
      break;
    }
  }
}

void SimRobot::sampleSonar() {
  // Base-frame angles of the three HC-SR04s (front, left, right). Base frame
  // is x forward, y left, so left = +pi/2.
  const double baseAng[3] = {0.0, kPi_ / 2.0, -kPi_ / 2.0};
  for (int s = 0; s < 3; ++s) {
    if (timeS_ - lastSonarS_[s] < kSonarPeriodS) continue;
    lastSonarS_[s] = timeS_;
    // A real HC-SR04 has a ~15° cone; nearest return in the cone wins.
    double best = kSonarMaxM;
    for (int k = -2; k <= 2; ++k) {
      const double ang = s_.truePose.theta + baseAng[s] + k * 0.06;
      best = std::min(best, world_.raycast(s_.truePose.translation(), ang,
                                           kSonarMaxM));
    }
    best += gaussian(noise_.sonarNoiseStd);
    s_.sonarM[s] = (best >= kSonarMaxM || best <= 0.02) ? 0.0 : best;
  }
}

}  // namespace vacuum
