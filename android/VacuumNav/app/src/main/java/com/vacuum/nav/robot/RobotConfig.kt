package com.vacuum.nav.robot

import org.json.JSONObject

/**
 * All robot constants, mirrored from vacuum_core's RobotConfig (the C++ side
 * is the source of truth for the math; this is the editable UI copy). Values
 * land here from Settings and are persisted as JSON; see docs/ASSUMPTIONS.md §2
 * for where each default came from.
 */
data class RobotConfig(
    // drive geometry (hoverboard_base.yaml)
    val wheelRadiusM: Double = 0.09615,
    val trackWidthM: Double = 0.18, // UNVERIFIED in repo -> calibrate in Phase 2
    val ticksPerRev: Double = 90.0,
    // body
    val robotRadiusM: Double = 0.23,
    val inflateSoftMarginM: Double = 0.10,
    // Cleaning brush width: area stats = path length x brush (RunStats).
    // Kotlin-side for now; the C++ coverage planner owns its copy in Phase 7.
    val brushWidthM: Double = 0.25,
    // lidar mounting (raw bins are clockwise from the zero mark)
    val scanBins: Int = 360,
    val lidarMirror: Boolean = true,
    val lidarMountYawDeg: Double = 180.0,
    val lidarOffsetXM: Double = 0.0,
    val lidarOffsetYM: Double = 0.0,
    val lidarMinRangeM: Double = 0.12,
    val lidarMaxRangeM: Double = 8.0,
    // sonar
    val sonarFrontStopM: Double = 0.35,
    val sonarSideStopM: Double = 0.05,
    // limits
    val maxLinearMps: Double = 0.35,
    val maxAngularRps: Double = 1.2,
    // link
    val boardScanPort: Int = 9900,
    val boardCmdPort: Int = 9901,
    val boardTelemPort: Int = 9902,
    val commandRateHz: Int = 20,
    val telemetryTimeoutMs: Int = 300,
    val lidarTimeoutMs: Int = 1000,
    // battery policy
    val batteryLowV: Double = 35.5,
    val batteryCriticalV: Double = 34.5,
    val batteryFullV: Double = 41.0,
    val robotName: String = "VacuumBot",
) {
    /** JSON accepted by vacuum_core's FlatJson (flat object, no nesting). */
    fun toJson(): String = JSONObject().apply {
        put("scanBins", scanBins)
        put("wheelRadiusM", wheelRadiusM)
        put("trackWidthM", trackWidthM)
        put("ticksPerRev", ticksPerRev)
        put("robotRadiusM", robotRadiusM)
        put("inflateSoftMarginM", inflateSoftMarginM)
        put("lidarMirror", lidarMirror)
        put("lidarMountYawDeg", lidarMountYawDeg)
        put("lidarOffsetXM", lidarOffsetXM)
        put("lidarOffsetYM", lidarOffsetYM)
        put("lidarMinRangeM", lidarMinRangeM)
        put("lidarMaxRangeM", lidarMaxRangeM)
        put("sonarFrontStopM", sonarFrontStopM)
        put("sonarSideStopM", sonarSideStopM)
        put("maxLinearMps", maxLinearMps)
        put("maxAngularRps", maxAngularRps)
        put("boardScanPort", boardScanPort)
        put("boardCmdPort", boardCmdPort)
        put("boardTelemPort", boardTelemPort)
        put("commandRateHz", commandRateHz)
        put("telemetryTimeoutMs", telemetryTimeoutMs)
        put("lidarTimeoutMs", lidarTimeoutMs)
        put("batteryLowV", batteryLowV)
        put("batteryCriticalV", batteryCriticalV)
        put("batteryFullV", batteryFullV)
        put("robotName", robotName)
    }.toString()

    /**
     * Differential drive: (v m/s, omega rad/s) -> wheel RPM pair, clamped to
     * the configured limits. Positive = forward / CCW.
     */
    fun velocityToWheelRpm(v: Double, omega: Double): Pair<Double, Double> {
        val vClamped = v.coerceIn(-maxLinearMps, maxLinearMps)
        val wClamped = omega.coerceIn(-maxAngularRps, maxAngularRps)
        val vL = vClamped - wClamped * trackWidthM / 2.0
        val vR = vClamped + wClamped * trackWidthM / 2.0
        val mpsPerRpm = 2.0 * Math.PI * wheelRadiusM / 60.0
        return Pair(vL / mpsPerRpm, vR / mpsPerRpm)
    }
}
