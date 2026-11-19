package com.vacuum.nav.lidar

import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.robot.Scan
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlin.math.cos
import kotlin.math.sin

/** Which lidar behaviour owns the wheels. */
enum class LidarMode { MANUAL, AVOID, FOLLOW }

/** Everything the UI needs from the lidar layer, per frame. */
data class LidarState(
    val mode: LidarMode = LidarMode.MANUAL,
    val warning: WarningState = WarningState(
        false, 0.0, Double.POSITIVE_INFINITY, false, 0.0),
    val target: Target? = null,
    val bumper: BumperAction = BumperAction.NONE,
    val avoidanceWhy: String = "",
)

enum class BumperAction { NONE, SLOW, STOP }

/**
 * Stateful wrapper around the ported laws (robot_ws/hoverboard_laser):
 *
 *  - WARNING runs every tick in every mode (red UI + phone beep + HBC2
 *    buzzer flag on the real link). Facing-turn only when a behaviour owns
 *    the wheels, matching the reference's "paused while joystick active".
 *  - AVOID / FOLLOW own the wheels until the joystick overrides (the
 *    reference nodes pause on /JoyState the same way).
 *  - BUMPER: the repo's Nav2 collision_monitor polygons (param
 *    collision_monitor_hoverboard.yaml: stop x 0.20-0.32 |y|<=0.19, slow
 *    x 0.20-0.60 |y|<=0.24 at 40 %) applied as a hard override in ALL
 *    modes - the app-side equivalent of /cmd_vel -> /cmd_vel_safe, so even
 *    the joystick can't drive through it (prompt §7).
 */
class LidarBehaviorController(
    private val config: RobotConfig = RobotConfig(),
    private val avoidanceParams: AvoidanceParams = AvoidanceParams(),
    private val trackerParams: TrackerParams = TrackerParams(),
    private val warningParams: WarningParams = WarningParams(),
) {
    private val _state = MutableStateFlow(LidarState())
    val state: StateFlow<LidarState> = _state.asStateFlow()

    private val linPid = SinglePid(trackerParams.linKp, 0.0, trackerParams.linKd)
    private val angPid = SinglePid(trackerParams.angKp, 0.0, trackerParams.angKd)
    private val warnPid = SinglePid(warningParams.angKp, 0.0, warningParams.angKd)

    private var latestPoints: List<LidarPoint> = emptyList()
    private var holdUntilMs: Long = 0
    private var heldDecision: AvoidanceDecision? = null
    @Volatile private var mode: LidarMode = LidarMode.MANUAL
    @Volatile var buzzerEnabled: Boolean = true   // phone-side beep toggle

    fun setMode(m: LidarMode) {
        if (m != mode) {
            mode = m
            holdUntilMs = 0
            heldDecision = null
            linPid.reset()
            angPid.reset()
            _state.value = _state.value.copy(mode = m, target = null)
        }
    }

    fun currentMode(): LidarMode = mode

    /** Feed every new scan (6 Hz). Cheap: 360-point filters. */
    fun onScan(scan: Scan) {
        val pts = robotFramePoints(scan, config, avoidanceParams.minRange)
        latestPoints = pts
        val warn = warningDecision(pts, warningParams, warnPid)
        val target = if (mode == LidarMode.FOLLOW)
            selectTarget(pts, trackerParams) else null
        _state.value = _state.value.copy(
            warning = warn,
            target = target,
            bumper = bumperAction(pts),
        )
    }

    /**
     * Control tick (20 Hz). Returns the (v, w) the wheels should get in
     * m/s and rad/s, or null when MANUAL owns them (caller sends its own).
     * Always applies the bumper override when a behaviour is active; the
     * manual path should call [applyBumper] too.
     */
    fun compute(nowMs: Long): Pair<Double, Double>? {
        when (mode) {
            LidarMode.MANUAL -> return null
            LidarMode.AVOID -> {
                // Keep the held decision until its hold time expires
                // (reference: `time.monotonic() < hold_until` in on_scan).
                val decision = heldDecision?.takeIf { nowMs < holdUntilMs }
                    ?: avoidanceDecision(latestPoints, avoidanceParams).also {
                        heldDecision = it
                        holdUntilMs = nowMs + it.holdMs
                    }
                _state.value = _state.value.copy(avoidanceWhy = decision.why)
                return applyBumper(decision.v to decision.w)
            }
            LidarMode.FOLLOW -> {
                val t = selectTarget(latestPoints, trackerParams)
                _state.value = _state.value.copy(target = t)
                if (t == null) return null  // nothing in view: stop (reference
                // crashed on min() of an empty list; we just idle)
                val (v, w) = followControl(t, trackerParams, linPid, angPid)
                return applyBumper(v to w)
            }
        }
    }

    /**
     * Bumper for the manual path (joystick): scales/stops the requested
     * (v, w). Returns the safe command to actually send.
     */
    fun applyBumper(cmd: Pair<Double, Double>): Pair<Double, Double> {
        return when (_state.value.bumper) {
            BumperAction.STOP -> 0.0 to 0.0
            BumperAction.SLOW -> (cmd.first * 0.4) to cmd.second
            BumperAction.NONE -> cmd
        }
    }

    /**
     * collision_monitor_hoverboard.yaml polygons, in the base frame
     * (x forward, y left). Stop: any point in the front bumper window;
     * slow: the wider corridor at 40 % speed.
     */
    private fun bumperAction(points: List<LidarPoint>): BumperAction {
        var slow = false
        for (p in points) {
            val x = p.rangeM * cos(p.angleRad)
            val y = p.rangeM * sin(p.angleRad)
            if (y > 0.24 || y < -0.24) continue
            if (x in 0.20..0.60) slow = true
            if (x in 0.20..0.32) return BumperAction.STOP
        }
        return if (slow) BumperAction.SLOW else BumperAction.NONE
    }

    /** World position of the tracked target (for the map overlay). */
    fun targetWorld(robotX: Double, robotY: Double, robotTheta: Double):
        Pair<Double, Double>? {
        val t = _state.value.target ?: return null
        val ang = robotTheta + t.angleRad
        return (robotX + t.rangeM * cos(ang)) to (robotY + t.rangeM * sin(ang))
    }

    /** Rising/falling edge of the buzzer for the caller to beep. */
    private var lastBuzzer = false
    fun buzzerEdge(): Boolean? {
        val now = _state.value.warning.buzzer && buzzerEnabled
        val edge = if (now != lastBuzzer) now else null
        lastBuzzer = now
        return edge
    }
}
