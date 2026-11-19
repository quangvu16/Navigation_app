package com.vacuum.nav.robot

import com.vacuum.nav.ui.ScanTransform
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI

/**
 * Host-side JVM tests for the Kotlin math twins. The C++ side has the full
 * GoogleTest suite (vacuum_core/tests); these guard the same contracts in the
 * app layer: velocity -> wheel RPM and the scan bin -> world transform.
 */
class RobotConfigTest {

    @Test
    fun velocityToWheelRpmStraightDrive() {
        val cfg = RobotConfig()
        val (l, r) = cfg.velocityToWheelRpm(0.2, 0.0)
        assertEquals(l, r, 1e-9)
        // 0.2 m/s on a 0.09615 m wheel = 0.2/(2*pi*r/60) rpm.
        val expected = 0.2 / (2.0 * PI * 0.09615 / 60.0)
        assertEquals(expected, l, 1e-6)
    }

    @Test
    fun velocityToWheelRpmClampsToLimits() {
        val cfg = RobotConfig()
        val (l, r) = cfg.velocityToWheelRpm(10.0, 0.0)   // absurd request
        val maxRpm = cfg.maxLinearMps / (2.0 * PI * cfg.wheelRadiusM / 60.0)
        // Straight drive stays straight at the clamp, both wheels at max.
        assertEquals(maxRpm, r, 1e-6)
        assertEquals(l, r, 1e-9)
    }

    @Test
    fun velocityToWheelRpmSpinInPlace() {
        val cfg = RobotConfig()
        val (l, r) = cfg.velocityToWheelRpm(0.0, 0.5)
        assertEquals(-r, l, 1e-9)
        assertTrue(r > 0)
    }
}

class ScanTransformTest {

    private val cfg = RobotConfig()  // mirror=true, mountYaw=180 (repo defaults)

    @Test
    fun binZeroPointsAtZeroMark() {
        // Zero mark = base +x rotated 180° -> world -x -> angle pi.
        val ang = ScanTransform.binToWorldAngle(cfg, 0.0, 0)
        assertEquals(PI, ang, 1e-9)
    }

    @Test
    fun binAnglesRunClockwiseWhenMirrored() {
        // Bin 1 is clockwise from bin 0: world angle decreases.
        val a0 = ScanTransform.binToWorldAngle(cfg, 0.0, 0)
        val a1 = ScanTransform.binToWorldAngle(cfg, 0.0, 1)
        assertTrue(a1 < a0)
    }

    @Test
    fun noReturnYieldsNullPoint() {
        assertNull(ScanTransform.binToWorldPoint(cfg, 1.0, 1.0, 0.0, 0, 0.0))
    }

    @Test
    fun rangeLandsOnKnownRay() {
        // Robot at origin facing +x; bin for world angle 0° exists at
        // bin 180 (zero mark at 180°, clockwise 180 bins = -180° = 0°... i.e.
        // bin 180 points along +x). A 2 m return must land at (2, 0).
        val p = ScanTransform.binToWorldPoint(cfg, 0.0, 0.0, 0.0, 180, 2.0)
        assertNotNull(p)
        assertEquals(2.0, p!!.first, 1e-6)
        assertEquals(0.0, p.second, 1e-6)
    }

    @Test
    fun roundTripUnderEpsilon() {
        // World point -> implied range/angle -> world point again is the
        // renderer's contract: < 1 cm (prompt §6).
        val p = ScanTransform.binToWorldPoint(cfg, 1.2, -3.4, 0.7, 45, 3.3)!!
        val dx = p.first - 1.2
        val dy = p.second + 3.4
        val range = kotlin.math.hypot(dx, dy)
        val ang = ScanTransform.binToWorldAngle(cfg, 0.7, 45)
        val q1 = 1.2 + range * kotlin.math.cos(ang)
        val q2 = -3.4 + range * kotlin.math.sin(ang)
        assertTrue(kotlin.math.hypot(q1 - p.first, q2 - p.second) < 0.01)
    }
}
