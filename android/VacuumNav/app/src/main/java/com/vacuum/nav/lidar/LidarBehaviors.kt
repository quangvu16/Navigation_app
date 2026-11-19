package com.vacuum.nav.lidar

import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.robot.Scan
import com.vacuum.nav.ui.ScanTransform
import kotlin.math.abs
import kotlin.math.PI

/**
 * Lidar behaviours ported from the reference repo's `hoverboard_laser`
 * package (robot_ws/src/hoverboard_laser), which itself adapts Yahboom's
 * implementations. ROS is not used: the laws are plain functions over
 * robot-frame points, unit-tested on the host JVM.
 *
 *   laser_Avoidance.py  -> avoidanceDecision()  (sector table + hold)
 *   laser_Tracker.py    -> selectTarget() + followControl()  (tracking +
 *                          following: PID keep-distance)
 *   laser_Warning.py    -> warningDecision()    (face nearest + buzzer)
 *   common.py           -> SinglePid, robotFramePoints()
 *
 * Angle convention (common.py): robot frame, 0 = straight ahead, positive =
 * LEFT (CCW), wrapped to (-180, 180]. Our bin->base transform is the same
 * convention once the mount yaw is included (bin0 points at the zero mark =
 * base +x rotated by lidarMountYawDeg, which is 180 deg on this robot).
 */

// ---------------------------------------------------------------- common.py
data class LidarPoint(val angleRad: Double, val rangeM: Double)

/** Yahboom's per-sample PID (no dt) with an output clamp - common.py. */
class SinglePid(
    var kp: Double,
    var ki: Double = 0.0,
    var kd: Double,
    var limit: Double = Double.POSITIVE_INFINITY,
) {
    private var integral = 0.0
    private var prevError = 0.0

    fun compute(target: Double, current: Double): Double {
        val error = target - current
        integral += error
        val derivative = error - prevError
        prevError = error
        val out = kp * error + ki * integral + kd * derivative
        return out.coerceIn(-limit, limit)
    }

    fun reset() {
        integral = 0.0
        prevError = 0.0
    }
}

/**
 * Valid returns as robot-frame points. `minRange` drops the chassis
 * (reference: 0.30 m; our own pre-filter already removed < 0.12 m body
 * returns, but the behaviours keep their own window so their thresholds
 * mean exactly what they meant in the reference).
 */
fun robotFramePoints(
    scan: Scan,
    config: RobotConfig,
    minRange: Double,
): List<LidarPoint> {
    val out = ArrayList<LidarPoint>(scan.ranges.size)
    val mountYaw = Math.toRadians(config.lidarMountYawDeg)
    for (i in scan.ranges.indices) {
        val r = scan.ranges[i].toDouble()
        if (r <= 0.0 || r < minRange) continue
        val base = mountYaw + ScanTransform.binToBaseAngle(config, i)
        var a = (base + PI) % (2.0 * PI)
        if (a <= 0) a += 2.0 * PI
        out.add(LidarPoint(a - PI, r))
    }
    return out
}

// ---------------------------------------------------------- laser_Avoidance
data class AvoidanceParams(
    val linear: Double = 0.2,        // m/s (reference default)
    val angular: Double = 0.8,       // rad/s
    val sectorDeg: Double = 45.0,    // LaserAngle: side sectors span 20..45
    val frontDeg: Double = 20.0,     // front sector |a| <= 20
    val responseDist: Double = 0.55, // ResponseDist
    val minPoints: Int = 5,          // blocked when > min_points in range
    val minRange: Double = 0.30,
) {
    /** Blocked window: ResponseDist * 1.5, exactly as the reference. */
    val blockRange: Double get() = responseDist * 1.5
}

data class AvoidanceDecision(
    val v: Double,       // m/s, + = forward
    val w: Double,       // rad/s, + = CCW
    val why: String,
    val holdMs: Long,    // hold this decision before re-evaluating
)

/**
 * The reference decision table (laser_Avoidance.py lines 62-77), including
 * the fix noted there: a blocked front never drives forward into it.
 * `holdMs` maps the reference's hold seconds (0.2 / 0.4).
 */
fun avoidanceDecision(
    points: List<LidarPoint>,
    p: AvoidanceParams = AvoidanceParams(),
): AvoidanceDecision {
    val near = points.filter { it.rangeM < p.blockRange }
    val sec = Math.toRadians(p.sectorDeg)
    val fr = Math.toRadians(p.frontDeg)
    val front = near.count { abs(it.angleRad) <= fr } > p.minPoints
    val left = near.count { it.angleRad > fr && it.angleRad < sec } > p.minPoints
    val right = near.count { it.angleRad < -fr && it.angleRad > -sec } > p.minPoints
    return when {
        front && left && right -> AvoidanceDecision(0.0, -p.angular,
            "boxed in: turn right on the spot", 400L)
        front && right -> AvoidanceDecision(p.linear * 0.5, p.angular,
            "front+right blocked: turn left", 200L)
        front && left -> AvoidanceDecision(p.linear * 0.5, -p.angular,
            "front+left blocked: turn right", 200L)
        front -> AvoidanceDecision(0.0, p.angular,
            "front blocked: turn left", 200L)
        left && right -> AvoidanceDecision(p.linear * 0.5, -p.angular,
            "narrow gap: turn right", 400L)
        left -> AvoidanceDecision(p.linear, -p.angular,
            "left blocked: veer right", 200L)
        right -> AvoidanceDecision(p.linear, p.angular,
            "right blocked: veer left", 200L)
        else -> AvoidanceDecision(p.linear, 0.0, "clear: forward", 0L)
    }
}

// ------------------------------------------------- laser_Tracker (tracking)
data class TrackerParams(
    val priorityDeg: Double = 10.0,  // priorityAngle
    val windowDeg: Double = 30.0,    // LaserAngle
    val responseDist: Double = 0.55, // ResponseDist to hold
    val maxLinear: Double = 0.25,    // max_linear
    val maxAngular: Double = 0.8,    // max_angular
    val minRange: Double = 0.30,
    val linKp: Double = 2.0, val linKd: Double = 2.0,
    val angKp: Double = 3.0, val angKd: Double = 5.0,
)

/** The object being tracked: nearest return in the follow window. */
data class Target(val angleRad: Double, val rangeM: Double)

/**
 * Target selection (laser_Tracker.py lines 49-57): anything inside
 * priorityAngle of straight ahead AND within responseDist + 0.5 wins;
 * otherwise the nearest return inside windowDeg. Null = nothing to follow.
 */
fun selectTarget(points: List<LidarPoint>, p: TrackerParams): Target? {
    val prio = Math.toRadians(p.priorityDeg)
    val win = Math.toRadians(p.windowDeg)
    val candidates = points.filter { it.rangeM >= p.minRange }
    val front = candidates.filter {
        abs(it.angleRad) < prio && it.rangeM < p.responseDist + 0.5
    }
    val pick = if (front.isNotEmpty()) front
    else candidates.filter { abs(it.angleRad) < win }
    if (pick.isEmpty()) return null
    val best = pick.minBy { it.rangeM }
    return Target(best.angleRad, best.rangeM)
}

/**
 * Follow control (laser_Tracker.py lines 58-68): drive to hold
 * ResponseDist, turn to face the target; the angular deadband (|w| < 0.1
 * -> 0) stops the robot dithering when nearly aligned.
 */
fun followControl(
    target: Target,
    p: TrackerParams,
    linPid: SinglePid,
    angPid: SinglePid,
): Pair<Double, Double> {
    linPid.limit = p.maxLinear
    angPid.limit = p.maxAngular
    var dist = target.rangeM
    // Reference snap: within 0.1 m of the setpoint, treat as exactly there
    // (kills integral/derivative buzz at the target distance).
    if (abs(dist - p.responseDist) < 0.1) dist = p.responseDist
    // v = -lin_pid(dist, min_dist): too far (min_dist > dist) -> error < 0
    // -> -error > 0 -> forward; too close -> back up.
    val v = -linPid.compute(p.responseDist, dist)
    // w = ang_pid(min_ang/48, 0): target left (+ang) -> positive w (CCW).
    // NOTE: the reference's scan_points returns DEGREES, so the /48 scaling
    // is degrees-based (6 deg -> 0.125 error -> strong turn). Porting it
    // with radians would put every small bearing under the 0.1 deadband.
    var w = angPid.compute(Math.toDegrees(target.angleRad) / 48.0, 0.0)
    if (abs(w) < 0.1) w = 0.0
    return v to w
}

// --------------------------------------------------------- laser_Warning
data class WarningParams(
    val windowDeg: Double = 10.0,    // LaserAngle
    val responseDist: Double = 0.5,  // buzzer threshold (reference note:
                                     // Yahboom's 0.3 is inside our body
                                     // filter, so 0.5 here)
    val maxAngular: Double = 0.8,
    val turnToFace: Boolean = true,
    val minRange: Double = 0.30,
    val angKp: Double = 3.0, val angKd: Double = 5.0,
)

data class WarningState(
    val active: Boolean,        // something inside the +-window
    val nearestAngleRad: Double,
    val nearestDistM: Double,
    val buzzer: Boolean,        // nearest <= responseDist
    val turnRate: Double,       // rad/s to face it (0 when turnToFace off)
)

/**
 * Warning law (laser_Warning.py lines 55-67): nearest return inside
 * +-LaserAngle; face it with the angular PID (deadband 0.1), buzzer when
 * closer than ResponseDist.
 */
fun warningDecision(
    points: List<LidarPoint>,
    p: WarningParams = WarningParams(),
    angPid: SinglePid = SinglePid(p.angKp, 0.0, p.angKd),
): WarningState {
    val win = Math.toRadians(p.windowDeg)
    val inWindow = points.filter {
        abs(it.angleRad) < win && it.rangeM >= p.minRange
    }
    if (inWindow.isEmpty())
        return WarningState(false, 0.0, Double.POSITIVE_INFINITY, false, 0.0)
    val nearest = inWindow.minBy { it.rangeM }
    var turn = 0.0
    if (p.turnToFace) {
        angPid.limit = p.maxAngular
        // Degrees / 48, exactly as laser_Warning.py (see followControl note).
        turn = angPid.compute(Math.toDegrees(nearest.angleRad) / 48.0, 0.0)
        if (abs(turn) < 0.1) turn = 0.0
    }
    return WarningState(
        active = true,
        nearestAngleRad = nearest.angleRad,
        nearestDistM = nearest.rangeM,
        buzzer = nearest.rangeM <= p.responseDist,
        turnRate = turn,
    )
}
