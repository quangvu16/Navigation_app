package com.vacuum.nav.lidar

import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.robot.Scan
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.abs

/**
 * Ports of robot_ws/hoverboard_laser (laser_Avoidance / laser_Tracker /
 * laser_Warning + common.py) - the decision table, target selection, PID
 * directions and warning thresholds are asserted against the reference
 * behaviour.
 */
class LidarBehaviorsTest {

    private val cfg = RobotConfig()

    private fun scanOf(binRanges: Map<Int, Float>): Scan {
        val r = FloatArray(cfg.scanBins)
        binRanges.forEach { (bin, m) -> r[bin] = m }
        return Scan(r, ByteArray(cfg.scanBins), 1, 0.0, 0.16, 6f)
    }

    private fun pts(vararg pairs: Pair<Double, Double>): List<LidarPoint> =
        pairs.map { (a, r) -> LidarPoint(Math.toRadians(a), r) }

    // ------------------------------------------------------- angle convention
    @Test
    fun robotFrameAnglesMatchWireConvention() {
        // Default config: mirror=true, mountYaw=180 -> bin180 points ahead,
        // bin90 to the LEFT (+90 deg), bin270 to the right.
        val scan = scanOf(mapOf(180 to 1.0f, 90 to 2.0f, 270 to 3.0f, 0 to 4.0f))
        val points = robotFramePoints(scan, cfg, minRange = 0.30)
        assertEquals(4, points.size)
        val byRange = points.associateBy { it.rangeM }
        assertEquals(0.0, byRange.getValue(1.0).angleRad, 1e-9)      // ahead
        assertEquals(PI / 2, byRange.getValue(2.0).angleRad, 1e-9)   // left
        assertEquals(-PI / 2, byRange.getValue(3.0).angleRad, 1e-9)  // right
        assertEquals(PI, abs(byRange.getValue(4.0).angleRad), 1e-9)  // behind
    }

    @Test
    fun robotFramePointsDropsBodyReturns() {
        val scan = scanOf(mapOf(180 to 0.10f, 90 to 0.25f, 270 to 0.50f))
        val points = robotFramePoints(scan, cfg, minRange = 0.30)
        assertEquals(1, points.size)   // only the 0.50 m return survives
        assertEquals(0.50, points[0].rangeM, 1e-9)
    }

    // ---------------------------------------------------------- avoidance
    @Test
    fun avoidanceDecisionTableMatchesReference() {
        val p = AvoidanceParams()
        val blocked = 6   // > min_points

        // front |a| <= 20, left 20..45, right -45..-20; all within 0.825 m
        fun front() = (0 until blocked).map { pts(0.0 to 0.6)[0] }
        fun left() = (0 until blocked).map { LidarPoint(Math.toRadians(30.0), 0.6) }
        fun right() = (0 until blocked).map { LidarPoint(Math.toRadians(-30.0), 0.6) }

        avoidanceDecision(front() + left() + right(), p).let {
            assertEquals(0.0, it.v, 1e-9)
            assertEquals(-p.angular, it.w, 1e-9)
            assertEquals(400L, it.holdMs)
        }
        avoidanceDecision(front() + right(), p).let {
            assertEquals(p.linear * 0.5, it.v, 1e-9)
            assertEquals(+p.angular, it.w, 1e-9)
            assertEquals(200L, it.holdMs)
        }
        avoidanceDecision(front() + left(), p).let {
            assertEquals(p.linear * 0.5, it.v, 1e-9)
            assertEquals(-p.angular, it.w, 1e-9)
        }
        avoidanceDecision(front(), p).let {
            assertEquals(0.0, it.v, 1e-9)     // blocked front never drives forward
            assertEquals(+p.angular, it.w, 1e-9)
        }
        avoidanceDecision(left() + right(), p).let {
            assertEquals(p.linear * 0.5, it.v, 1e-9)
            assertEquals(-p.angular, it.w, 1e-9)
            assertEquals(400L, it.holdMs)
        }
        avoidanceDecision(left(), p).let {
            assertEquals(p.linear, it.v, 1e-9)
            assertEquals(-p.angular, it.w, 1e-9)
        }
        avoidanceDecision(right(), p).let {
            assertEquals(p.linear, it.v, 1e-9)
            assertEquals(+p.angular, it.w, 1e-9)
        }
        avoidanceDecision(emptyList(), p).let {
            assertEquals(p.linear, it.v, 1e-9)
            assertEquals(0.0, it.w, 1e-9)
            assertEquals(0L, it.holdMs)
        }
    }

    @Test
    fun avoidanceIgnoresPointsBeyondBlockRange() {
        //5 points at 1.0 m (beyond 0.55*1.5 = 0.825) must NOT block.
        val points = (0 until 5).map { LidarPoint(0.0, 1.0) }
        val d = avoidanceDecision(points)
        assertEquals(d.v, AvoidanceParams().linear, 1e-9)
        assertEquals(0.0, d.w, 1e-9)
    }

    // ------------------------------------------------- tracking + following
    @Test
    fun targetSelectionPrefersPriorityWindow() {
        val p = TrackerParams()
        val points = pts(
            5.0 to 0.9,     // inside priority (|a|<10, r < 0.55+0.5)
            25.0 to 0.6,    // closer, but only inside the wide window
        )
        val t = selectTarget(points, p)
        assertNotNull(t)
        assertEquals(Math.toRadians(5.0), t!!.angleRad, 1e-9)
        assertEquals(0.9, t.rangeM, 1e-9)
    }

    @Test
    fun targetSelectionFallsBackToWindow() {
        val p = TrackerParams()
        val points = pts(25.0 to 1.4, -20.0 to 1.1)   // none in priority
        val t = selectTarget(points, p)
        assertNotNull(t)
        assertEquals(1.1, t!!.rangeM, 1e-9)           // nearest in window
        assertEquals(Math.toRadians(-20.0), t.angleRad, 1e-9)
        val none = selectTarget(pts(80.0 to 0.5), p)  // outside window
        assertNull(none)
    }

    @Test
    fun followControlDrivesTowardAndFacesTarget() {
        val p = TrackerParams()
        val lin = SinglePid(p.linKp, 0.0, p.linKd)
        val ang = SinglePid(p.angKp, 0.0, p.angKd)

        // Too far: forward (v > 0).
        var (v, w) = followControl(Target(Math.toRadians(5.0), 1.5), p, lin, ang)
        assertTrue("too far must drive forward", v > 0)
        assertTrue("target left must turn CCW", w > 0)

        // Too close: back up (v < 0).
        lin.reset(); ang.reset()
        v = followControl(Target(0.0, 0.30), p, lin, ang).first
        assertTrue("too close must reverse", v < 0)

        // Deadband: target dead ahead -> w == 0 (no dithering).
        lin.reset(); ang.reset()
        w = followControl(Target(0.0, 0.55), p, lin, ang).second
        assertEquals(0.0, w, 1e-9)

        // Limits: PID output clamped to max_linear / max_angular.
        lin.reset(); ang.reset()
        val far = followControl(Target(Math.toRadians(40.0), 3.0), p, lin, ang)
        assertTrue(far.first <= p.maxLinear + 1e-9)
        assertTrue(far.second <= p.maxAngular + 1e-9)
    }

    // -------------------------------------------------------------- warning
    @Test
    fun warningBuzzerThresholdAndFacing() {
        val p = WarningParams()

        // Inside window, closer than 0.5 m -> buzzer + face it.
        val close = warningDecision(pts(6.0 to 0.40), p)
        assertTrue(close.active)
        assertTrue(close.buzzer)
        assertTrue("face the obstacle (left -> CCW)", close.turnRate > 0)

        // Inside window but beyond 0.5 m -> active, no buzzer.
        val far = warningDecision(pts(-6.0 to 0.60), p)
        assertTrue(far.active)
        assertFalse(far.buzzer)
        assertTrue("face right -> CW", far.turnRate < 0)

        // Outside +-10 deg window -> inactive, no buzzer.
        val side = warningDecision(pts(45.0 to 0.20), p)
        assertFalse(side.active)
        assertFalse(side.buzzer)
    }

    // --------------------------------------------------------------- bumper
    @Test
    fun bumperPolygonsMatchCollisionMonitor() {
        val c = LidarBehaviorController(cfg)

        // Stop polygon: x in [0.20, 0.32], |y| <= 0.19 (head-on ~30 cm).
        c.onScan(scanOf(mapOf(180 to 0.30f)))
        assertEquals(BumperAction.STOP, c.state.value.bumper)

        // Slow polygon: x in [0.20, 0.60], |y| <= 0.24 (head-on 50 cm).
        c.onScan(scanOf(mapOf(180 to 0.50f)))
        assertEquals(BumperAction.SLOW, c.state.value.bumper)

        // Same range but off to the side (y > 0.24): outside the corridor.
        c.onScan(scanOf(mapOf(90 to 0.50f)))   // straight left -> y = 0.5
        assertEquals(BumperAction.NONE, c.state.value.bumper)

        // Nothing close: clear.
        c.onScan(scanOf(mapOf(180 to 2.0f)))
        assertEquals(BumperAction.NONE, c.state.value.bumper)
    }

    @Test
    fun bumperStopsManualJoystickCommands() {
        val c = LidarBehaviorController(cfg)
        c.onScan(scanOf(mapOf(180 to 0.30f)))   // STOP polygon hit
        val (v, w) = c.applyBumper(0.3 to 1.0)
        assertEquals(0.0, v, 1e-9)
        assertEquals(0.0, w, 1e-9)
    }

    // ------------------------------------------------------------- modes
    @Test
    fun joystickStyleOverrideSwitchesModeToManual() {
        val c = LidarBehaviorController(cfg)
        c.setMode(LidarMode.FOLLOW)
        assertEquals(LidarMode.FOLLOW, c.currentMode())
        // Nothing inside the follow window (30 deg): the only return is
        // 90 deg to the left -> no target (reference: stop, no min() crash).
        c.onScan(scanOf(mapOf(90 to 1.0f)))
        assertNull(c.state.value.target)
        assertNull(c.compute(0L))
    }

    @Test
    fun followModeProducesControlFromScan() {
        val c = LidarBehaviorController(cfg)
        c.setMode(LidarMode.FOLLOW)
        // Object dead ahead at 1.5 m (in the30 deg window).
        c.onScan(scanOf(mapOf(180 to 1.5f, 179 to 1.6f, 181 to 1.6f)))
        val cmd = c.compute(0L)
        assertNotNull(cmd)
        assertTrue("drive toward the target", cmd!!.first > 0)
        val t = c.state.value.target
        assertNotNull(t)
        assertEquals(1.5, t!!.rangeM, 1e-9)
    }

    @Test
    fun avoidModeHoldsDecisionBetweenScans() {
        val c = LidarBehaviorController(cfg)
        c.setMode(LidarMode.AVOID)
        // Front blocked (6 returns at 0.6 m dead ahead -> front sector).
        val ranges = (175..185).associate { it to 0.6f }
        c.onScan(scanOf(ranges))
        val first = c.compute(1000L)!!
        assertEquals(0.0, first.first, 1e-9)   // front blocked: no forward
        assertTrue(first.second > 0)           // turn left
        // Within the 200 ms hold the decision must not change even if the
        // world instantly clears (reference: hold_until in on_scan).
        c.onScan(scanOf(mapOf(180 to 5.0f)))
        val held = c.compute(1050L)!!
        assertEquals(first.first, held.first, 1e-9)
        assertEquals(first.second, held.second, 1e-9)
        // After the hold expires the clear scan wins.
        val after = c.compute(1300L)!!
        assertTrue(after.first > 0)   // clear: forward
    }
}
