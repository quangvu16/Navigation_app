package com.vacuum.nav.robot

import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow

/** World-frame robot pose (metres, radians CCW). */
data class Pose(val x: Float, val y: Float, val theta: Float) {
    companion object {
        val ZERO = Pose(0f, 0f, 0f)
    }
}

/**
 * Telemetry shaped like HBT1/HBT2 (docs/PROTOCOL.md) so the UI and the
 * engine state machine see identical data from the simulator and, later,
 * the real UDP link.
 */
data class Telemetry(
    val pose: Pose = Pose.ZERO,          // map/true pose (sim) or EKF pose later
    val odomPose: Pose = Pose.ZERO,      // raw tick odometry
    val gyroZ: Float = 0f,
    val batteryV: Float = 0f,
    val rpmL: Float = 0f,
    val rpmR: Float = 0f,
    val ticksL: Long = 0,
    val ticksR: Long = 0,
    val sonarFrontM: Float = 0f,
    val sonarLeftM: Float = 0f,
    val sonarRightM: Float = 0f,
    val timestampMs: Long = 0L,
)

/** One lidar revolution in the wire convention (bin i clockwise from zero). */
data class Scan(
    val ranges: FloatArray,        // metres, 0f = no return
    val intensity: ByteArray,      // 0..255 per bin
    val scanId: Long,
    val startS: Double,
    val endS: Double,
    val rotationHz: Float,
)

/**
 * The one door to a robot — simulated or real. Phase 1 ships SimRobotLink;
 * Phase 2 adds UdpRobotLink speaking LDS2/HBT2/HBC2 (docs/PROTOCOL.md) and
 * the control loop cannot tell the difference.
 */
interface RobotLink {
    val telemetry: StateFlow<Telemetry>
    val scans: SharedFlow<Scan>

    /** Wheel speed targets in RPM (+ = forward). Zero on stop/e-stop paths. */
    fun setWheelRpm(rpmL: Float, rpmR: Float)

    fun close()
}
