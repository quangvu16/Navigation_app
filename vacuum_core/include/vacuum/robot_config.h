// robot_config.h — ALL robot constants in one struct, editable in Settings
// and persisted as JSON. Nothing else in vacuum_core may hard-code a robot
// number (wheel radius, ticks/rev, track width, lidar offset/mirroring,
// speed limits, ports); they all live here.
//
// Defaults come from the reference repo (docs/ASSUMPTIONS.md §2):
// hoverboard_base.yaml + firmware config.h. Ones the repo itself marks as
// unverified (track width) are flagged below and re-checkable in Diagnostics.

#pragma once

#include <string>

#include "vacuum/json.h"

namespace vacuum {

struct RobotConfig {
  // ---- drive geometry (hoverboard_base.yaml) ------------------------------
  double wheelRadiusM = 0.09615;   // measured 2026-09-26 ("19.23 cm wheel")
  double trackWidthM = 0.18;       // UNVERIFIED in the repo -> calibrate (Phase 2 wizard)
  double ticksPerRev = 90.0;       // HB_TICKS_PER_REV: 15 pole pairs x 6 hall states

  // ---- body ---------------------------------------------------------------
  // Half-diagonal of the repo's ±0.165 m square footprint. Used as the costmap
  // inflation radius base and the coverage erosion radius.
  double robotRadiusM = 0.23;
  double inflateSoftMarginM = 0.10;  // extra soft cost margin beyond the body

  // ---- lidar mounting -----------------------------------------------------
  // Raw convention (verified in lidar_ld14.h): bin i is CLOCKWISE from the
  // lidar's zero mark. The decoder never reordered bins; these two settings
  // carry the whole sensor->base correction (the ROS setup used flip_lr=true,
  // angle0_deg=180 for the same job).
  int scanBins = 360;               // SCAN_BINS: 1 deg grid, <= 720 (one MTU)
  bool lidarMirror = true;          // true: bins run clockwise (negate angles)
  double lidarMountYawDeg = 180.0;  // zero mark relative to base +x
  double lidarOffsetXM = 0.0;       // laser frame origin in base frame
  double lidarOffsetYM = 0.0;
  double lidarMinRangeM = 0.12;     // below this = robot body / blind zone
  double lidarMaxRangeM = 8.0;      // LD14/LD14P hardware limit

  // ---- sonar (firmware config.h) -----------------------------------------
  double sonarFrontStopM = 0.35;
  double sonarSideStopM = 0.05;

  // ---- speed limits (app policy; never above repo max_wheel_speed_mps=0.5) --
  double maxLinearMps = 0.35;
  double maxAngularRps = 1.2;

  // ---- link / protocol ----------------------------------------------------
  int boardScanPort = 9900;   // v1 DEST_PORT / v2 default scan_port
  int boardCmdPort = 9901;    // v1 HB_CMD_PORT / v2 claim+commands
  int boardTelemPort = 9902;  // v1 HB_TELEM_PORT / v2 default telem_port
  int commandRateHz = 20;
  int telemetryTimeoutMs = 300;  // no HBT for this long -> ERROR_LINK + zero RPM
  int lidarTimeoutMs = 1000;     // no scan for this long -> zero RPM

  // ---- battery policy (battery.h thresholds; 10S pack) --------------------
  double batteryLowV = 35.5;     // speed halved on the board
  double batteryCriticalV = 34.5;  // wheels held at 0 on the board
  double batteryFullV = 41.0;    // for the % display

  // ---- misc ---------------------------------------------------------------
  std::string robotName = "VacuumBot";

  // Meters of travel per wheel tick: 2*pi*r / ticksPerRev (~6.71 mm).
  double metersPerTick() const { return 2.0 * 3.14159265358979323846 * wheelRadiusM / ticksPerRev; }

  FlatJson toJson() const {
    FlatJson j;
    j.set("scanBins", static_cast<double>(scanBins));
    j.set("wheelRadiusM", wheelRadiusM);
    j.set("trackWidthM", trackWidthM);
    j.set("ticksPerRev", ticksPerRev);
    j.set("robotRadiusM", robotRadiusM);
    j.set("inflateSoftMarginM", inflateSoftMarginM);
    j.set("lidarMirror", lidarMirror);
    j.set("lidarMountYawDeg", lidarMountYawDeg);
    j.set("lidarOffsetXM", lidarOffsetXM);
    j.set("lidarOffsetYM", lidarOffsetYM);
    j.set("lidarMinRangeM", lidarMinRangeM);
    j.set("lidarMaxRangeM", lidarMaxRangeM);
    j.set("sonarFrontStopM", sonarFrontStopM);
    j.set("sonarSideStopM", sonarSideStopM);
    j.set("maxLinearMps", maxLinearMps);
    j.set("maxAngularRps", maxAngularRps);
    j.set("boardScanPort", boardScanPort);
    j.set("boardCmdPort", boardCmdPort);
    j.set("boardTelemPort", boardTelemPort);
    j.set("commandRateHz", commandRateHz);
    j.set("telemetryTimeoutMs", telemetryTimeoutMs);
    j.set("lidarTimeoutMs", lidarTimeoutMs);
    j.set("batteryLowV", batteryLowV);
    j.set("batteryCriticalV", batteryCriticalV);
    j.set("batteryFullV", batteryFullV);
    j.set("robotName", robotName);
    return j;
  }

  static RobotConfig fromJson(const FlatJson& j) {
    RobotConfig c;
    c.scanBins = static_cast<int>(j.num("scanBins", c.scanBins));
    c.wheelRadiusM = j.num("wheelRadiusM", c.wheelRadiusM);
    c.trackWidthM = j.num("trackWidthM", c.trackWidthM);
    c.ticksPerRev = j.num("ticksPerRev", c.ticksPerRev);
    c.robotRadiusM = j.num("robotRadiusM", c.robotRadiusM);
    c.inflateSoftMarginM = j.num("inflateSoftMarginM", c.inflateSoftMarginM);
    c.lidarMirror = j.boolean("lidarMirror", c.lidarMirror);
    c.lidarMountYawDeg = j.num("lidarMountYawDeg", c.lidarMountYawDeg);
    c.lidarOffsetXM = j.num("lidarOffsetXM", c.lidarOffsetXM);
    c.lidarOffsetYM = j.num("lidarOffsetYM", c.lidarOffsetYM);
    c.lidarMinRangeM = j.num("lidarMinRangeM", c.lidarMinRangeM);
    c.lidarMaxRangeM = j.num("lidarMaxRangeM", c.lidarMaxRangeM);
    c.sonarFrontStopM = j.num("sonarFrontStopM", c.sonarFrontStopM);
    c.sonarSideStopM = j.num("sonarSideStopM", c.sonarSideStopM);
    c.maxLinearMps = j.num("maxLinearMps", c.maxLinearMps);
    c.maxAngularRps = j.num("maxAngularRps", c.maxAngularRps);
    c.boardScanPort = static_cast<int>(j.num("boardScanPort", c.boardScanPort));
    c.boardCmdPort = static_cast<int>(j.num("boardCmdPort", c.boardCmdPort));
    c.boardTelemPort = static_cast<int>(j.num("boardTelemPort", c.boardTelemPort));
    c.commandRateHz = static_cast<int>(j.num("commandRateHz", c.commandRateHz));
    c.telemetryTimeoutMs = static_cast<int>(j.num("telemetryTimeoutMs", c.telemetryTimeoutMs));
    c.lidarTimeoutMs = static_cast<int>(j.num("lidarTimeoutMs", c.lidarTimeoutMs));
    c.batteryLowV = j.num("batteryLowV", c.batteryLowV);
    c.batteryCriticalV = j.num("batteryCriticalV", c.batteryCriticalV);
    c.batteryFullV = j.num("batteryFullV", c.batteryFullV);
    c.robotName = j.str("robotName", c.robotName);
    return c;
  }
};

}  // namespace vacuum
