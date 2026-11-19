package com.vacuum.nav.ui

import com.vacuum.nav.robot.RobotConfig
import kotlin.math.cos
import kotlin.math.sin

/**
 * Kotlin twin of vacuum_core's scan_transform.h — the ONLY place that turns
 * raw lidar bins into world points. Bin i is `i * (360/bins)` degrees
 * CLOCKWISE from the lidar's zero mark (wire convention); RobotConfig's
 * mirror + mountYaw carry the sensor-to-base correction. Keep in step with
 * the C++ header (its tests are the reference).
 */
object ScanTransform {
    /** Direction of bin `i` in the base frame (radians CCW from base +x). */
    fun binToBaseAngle(config: RobotConfig, bin: Int): Double {
        val step = 2.0 * Math.PI / config.scanBins
        val a = bin * step
        return if (config.lidarMirror) -a else a
    }

    /** World-frame direction of bin `i`, normalized to (-pi, pi]. */
    fun binToWorldAngle(config: RobotConfig, baseTheta: Double, bin: Int): Double {
        val raw = baseTheta + Math.toRadians(config.lidarMountYawDeg) +
            binToBaseAngle(config, bin)
        var a = (raw + Math.PI) % (2.0 * Math.PI)
        if (a <= 0) a += 2.0 * Math.PI
        return a - Math.PI
    }

    /**
     * World point measured by bin `i`, or null for a no-return (range <= 0).
     * Coordinate convention: world x right / y up (see docs/ARCHITECTURE.md).
     */
    fun binToWorldPoint(
        config: RobotConfig,
        baseX: Double,
        baseY: Double,
        baseTheta: Double,
        bin: Int,
        rangeM: Double,
    ): Pair<Double, Double>? {
        if (rangeM <= 0.0) return null
        // Lidar offset rotated into the world.
        val c = cos(baseTheta)
        val s = sin(baseTheta)
        val ox = baseX + config.lidarOffsetXM * c - config.lidarOffsetYM * s
        val oy = baseY + config.lidarOffsetXM * s + config.lidarOffsetYM * c
        val ang = binToWorldAngle(config, baseTheta, bin)
        return Pair(ox + rangeM * cos(ang), oy + rangeM * sin(ang))
    }
}
