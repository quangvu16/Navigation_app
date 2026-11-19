// sim_robot.h — the simulated robot: differential-drive dynamics with wheel
// slip and first-order motor lag, a biased/noisy gyro, a 360-beam lidar that
// sweeps at ~6 Hz (beams acquired progressively during the revolution, so
// scans carry motion distortion for de-skew tests), and 3x simulated sonar.
//
// Fidelity target: good enough to develop SLAM/localization/planning against
// before the hardware exists — deterministic with a fixed seed, noisy enough
// that filters cannot cheat.
//
// Conventions:
//   - True pose in the world frame (x right / y up, theta CCW).
//   - Scan ranges use the LDS wire convention: bin i is CLOCKWISE from the
//     lidar zero mark, 0.0 = no return. The lidar zero mark sits at
//     (base theta + lidarMountYawDeg), mirrored per RobotConfig — exactly
//     what the real firmware produces, so the app-side transform code is
//     exercised identically in sim and on hardware.
//   - Sonar: metres, 0.0 = no echo (the HC-SR04 "nothing came back" result).

#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "sim_world.h"
#include "vacuum/robot_config.h"

namespace vacuum {

// All noise knobs live here so tests can zero them for exactness checks.
struct SimNoise {
  double motorTauS = 0.12;        // first-order lag of wheel speed toward target
  double wheelSpeedNoiseStd = 0.02;   // rad/s at the wheel
  double slipStd = 0.015;         // ground slip fraction (random walk)
  double slipWalkStd = 0.002;     // slip fraction drift per second
  double gyroBiasStd = 0.005;     // rad/s, drawn at construction
  double gyroNoiseStd = 0.01;     // rad/s white noise per sample
  double rangeNoiseStd = 0.005;   // m
  double sonarNoiseStd = 0.01;    // m
  // false = deterministic ideal physics: no noise, no slip, no motor lag.
  bool enabled = true;
};

struct SimSensors {
  Pose2D truePose;      // ground truth
  Pose2D odomPose;      // integrated from wheel ticks (what /odom would say)
  double trueOmega = 0.0;   // rad/s, ground truth
  double gyroZ = 0.0;       // rad/s, what the IMU reports
  int32_t ticksL = 0, ticksR = 0;
  double rpmL = 0.0, rpmR = 0.0;      // measured wheel RPM (HBT rpm fields)
  double outRpmL = 0.0, outRpmR = 0.0;  // post-reflex/battery output RPM (HBT2)
  double batteryV = 39.0;

  // Latest completed scan (LDS wire convention).
  uint32_t scanId = 0;
  bool scanFresh = false;          // set during the step() in which a revolution
                                   // completed; cleared at the start of the next
  double scanStartS = 0.0, scanEndS = 0.0;
  double rotationHz = 6.0;
  std::vector<float> ranges;       // m, bin i clockwise from lidar zero; 0 = none
  std::vector<uint8_t> intensity;

  // Sonar in base frame; order front, left, right (HBT1 order).
  double sonarM[3] = {0.0, 0.0, 0.0};
};

class SimRobot {
 public:
  // noise = {} gives realistic noisy physics; tests pass
  // SimNoise{...enabled=false} for exactness checks.
  SimRobot(const RobotConfig& cfg, SimWorld world,
           const SimNoise& noise = SimNoise(), uint32_t seed = 1);

  // Wheel speed targets in RPM (+ = forward), as HBC2 would deliver them.
  void setWheelRpm(double rpmL, double rpmR);

  // Advance physics by dt seconds (call at a fixed rate, e.g. 200 Hz).
  void step(double dt);

  const SimSensors& sensors() const { return s_; }
  const SimWorld& world() const { return world_; }
  const RobotConfig& config() const { return cfg_; }

  // Force the true pose (teleport; used by tests and "set pose").
  void setPose(const Pose2D& p);

 private:
  void acquireBeams(double dt);
  void sampleSonar();
  double gaussian(double stddev);

  RobotConfig cfg_;
  SimWorld world_;
  SimNoise noise_;
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};

  double cmdRpmL_ = 0.0, cmdRpmR_ = 0.0;
  double wheelRadPerSL_ = 0.0, wheelRadPerSR_ = 0.0;  // filtered wheel speeds
  double slipFrac_ = 0.0;        // slowly drifting ground-slip fraction
  double gyroBias_ = 0.0;

  double scanRev_ = 0.0;         // fractional revolutions acquired so far
  double scanStartS_ = 0.0;
  uint16_t beamsDone_ = 0;
  double timeS_ = 0.0;
  double lastSonarS_[3] = {-1.0, -1.0, -1.0};

  SimSensors s_;
};

}  // namespace vacuum
