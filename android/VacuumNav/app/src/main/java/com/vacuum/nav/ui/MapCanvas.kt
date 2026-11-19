package com.vacuum.nav.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.rememberTransformableState
import androidx.compose.foundation.gestures.transformable
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import com.vacuum.nav.data.NoGoZone
import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.robot.Scan
import com.vacuum.nav.ui.theme.DangerRed
import com.vacuum.nav.ui.theme.LidarCyan
import com.vacuum.nav.ui.theme.MapUnknown
import com.vacuum.nav.ui.theme.RobotOrange
import kotlin.math.cos
import kotlin.math.sin

/**
 * The live map. World coordinates are metres (x right, y up); screen y is
 * down, so every draw flips Y (prompt §6 - goals and overlays always travel
 * in world metres, never pixels). Pan + pinch-zoom via one transformable
 * state; zones drawn as translucent red rectangles (Roborock-style no-go).
 */
@Composable
fun MapCanvas(
    config: RobotConfig,
    scan: Scan?,
    robotX: Float,
    robotY: Float,
    robotTheta: Float,
    sonar: FloatArray,
    modifier: Modifier = Modifier,
    zones: List<NoGoZone> = emptyList(),
    showGrid: Boolean = true,
    onScreenTransform: ((scale: Float, pxPerM: Float) -> Unit)? = null,
) {
    var scale by remember { mutableFloatStateOf(90f) }     // px per metre
    var offset by remember { mutableStateOf(Offset.Zero) } // pan, px

    val transformState = rememberTransformableState { zoomChange, panChange, _ ->
        scale = (scale * zoomChange).coerceIn(20f, 400f)
        offset += panChange
        onScreenTransform?.invoke(scale, scale)
    }

    Canvas(
        modifier = modifier
            .background(MapUnknown)
            .transformable(state = transformState),
    ) {
        val cx = size.width / 2f + offset.x
        val cy = size.height / 2f + offset.y

        fun toScreen(wx: Double, wy: Double): Offset =
            Offset(cx + (wx * scale).toFloat(), cy - (wy * scale).toFloat())

        if (showGrid) drawGrid(::toScreen)
        drawZones(zones, ::toScreen)

        val s = scan
        if (s != null) {
            val pts = s.ranges
            for (i in pts.indices) {
                val p = ScanTransform.binToWorldPoint(
                    config, robotX.toDouble(), robotY.toDouble(),
                    robotTheta.toDouble(), i, pts[i].toDouble(),
                ) ?: continue
                val sp = toScreen(p.first, p.second)
                drawCircle(color = LidarCyan, radius = 2.5f, center = sp)
            }
        }

        drawSonar(config, sonar, robotX, robotY, robotTheta, ::toScreen)

        // Robot: body disc + heading arrow (x forward).
        val rp = toScreen(robotX.toDouble(), robotY.toDouble())
        drawCircle(
            color = RobotOrange,
            radius = config.robotRadiusM.toFloat() * scale,
            center = rp,
            alpha = 0.9f,
        )
        val arrowLen = (config.robotRadiusM * 1.2).toFloat() * scale
        val tip = Offset(
            rp.x + arrowLen * cos(robotTheta),
            rp.y - arrowLen * sin(robotTheta),  // Y flip again
        )
        drawLine(color = Color.White, start = rp, end = tip, strokeWidth = 6f)
        val path = Path().apply {
            moveTo(tip.x, tip.y)
            lineTo(tip.x - 14f * cos(robotTheta - 0.5f),
                tip.y + 14f * sin(robotTheta - 0.5f))
            lineTo(tip.x - 14f * cos(robotTheta + 0.5f),
                tip.y + 14f * sin(robotTheta + 0.5f))
            close()
        }
        drawPath(path, Color.White)
    }
}

/** Exposes the current view transform so editors can hit-test touches. */
class MapViewTransform {
    var scale = 90f
    var cx = 0f
    var cy = 0f
    fun toScreen(wx: Double, wy: Double): Offset =
        Offset(cx + (wx * scale).toFloat(), cy - (wy * scale).toFloat())
    fun toWorld(sx: Float, sy: Float): Pair<Double, Double> =
        Pair((sx - cx) / scale.toDouble(), (cy - sy) / scale.toDouble())
}

private fun DrawScope.drawGrid(toScreen: (Double, Double) -> Offset) {
    val gridColor = Color.White.copy(alpha = 0.08f)
    for (i in -4..8) {
        drawLine(gridColor, toScreen(i * 0.5, -4.0), toScreen(i * 0.5, 8.0), 1f)
        drawLine(gridColor, toScreen(-4.0, i * 0.5), toScreen(8.0, i * 0.5), 1f)
    }
}

private fun DrawScope.drawZones(
    zones: List<NoGoZone>,
    toScreen: (Double, Double) -> Offset,
) {
    for (z in zones) {
        val tl = toScreen(z.x, z.y + z.h)   // world top-left -> screen
        val br = toScreen(z.x + z.w, z.y)
        drawRect(
            color = DangerRed.copy(alpha = 0.35f),
            topLeft = Offset(tl.x, tl.y),
            size = Size(br.x - tl.x, br.y - tl.y),
        )
        drawRect(
            color = DangerRed,
            topLeft = Offset(tl.x, tl.y),
            size = Size(br.x - tl.x, br.y - tl.y),
            style = Stroke(width = 3f),
        )
    }
}

private fun DrawScope.drawSonar(
    config: RobotConfig,
    sonar: FloatArray,
    robotX: Float,
    robotY: Float,
    robotTheta: Float,
    toScreen: (Double, Double) -> Offset,
) {
    val angles = doubleArrayOf(0.0, Math.PI / 2, -Math.PI / 2)
    for (i in 0..2) {
        val range = sonar[i]
        if (range <= 0f) continue
        val start = toScreen(robotX.toDouble(), robotY.toDouble())
        val mid = robotTheta + angles[i]
        val end = toScreen(
            robotX + range * cos(mid),
            robotY + range * sin(mid),
        )
        drawLine(
            color = DangerRed.copy(alpha = 0.4f),
            start = start, end = end, strokeWidth = 10f,
        )
    }
}
