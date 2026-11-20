package com.vacuum.nav.robot

import kotlinx.coroutines.flow.MutableStateFlow
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

    /** Short human description for Diagnostics ("SIM · simulator", ...). */
    val description: String get() = this::class.simpleName ?: "link"

    /** Link health - ERROR_LINK drives the app-wide banner. The simulator
     *  never errors; UDP/BLE override with their live flow. */
    val state: StateFlow<LinkState> get() = MutableStateFlow(LinkState.CLAIMED)

    /** Wheel speed targets in RPM (+ = forward). Zero on stop/e-stop paths. */
    fun setWheelRpm(rpmL: Float, rpmR: Float)

    /** Buzzer (laser_Warning's /Buzzer -> HBC2 flag bit1 on the real link;
     *  no-op in the simulator, which beeps on the phone instead). */
    fun setBuzzer(on: Boolean) {}

    /** Send a raw v2 datagram the control loop doesn't own (WFC1/WFO1,
     *  docs/PROTOCOL.md §3.9/§3.10). false = no such channel (SIM). */
    fun send(bytes: ByteArray): Boolean = false

    /** One-shot admin replies (WFCA/WFOA/WFOB). SIM never emits;
     *  UDP/BLE emit raw datagrams from the board. */
    val adminReplies: SharedFlow<ByteArray>

    fun close()
}
