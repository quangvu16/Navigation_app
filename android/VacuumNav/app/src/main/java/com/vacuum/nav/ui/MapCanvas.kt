package com.vacuum.nav.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.gestures.rememberTransformableState
import androidx.compose.foundation.gestures.transformable
import androidx.compose.ui.input.pointer.pointerInput
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

private const val COS30 = 0.8660254f

/**
 * Prebuilt occupancy layer (Phase 3): the bitmap is rebuilt only when the
 * mapper's version changes (prompt §6), never per frame. Unknown cells are
 * transparent so the dark map background shows through.
 */
data class GridImage(
    val image: androidx.compose.ui.graphics.ImageBitmap,
    val worldX: Double,      // bottom-left of the grid, world metres
    val worldY: Double,
    val sizeXM: Double,
    val sizeYM: Double,
)

/**
 * The live map. World coordinates are metres (x right, y up); screen y is
 * down, so every draw flips Y (prompt §6 - goals and overlays always travel
 * in world metres, never pixels). Pan + pinch-zoom via one transformable
 * state; zones drawn as translucent red rectangles (Roborock-style no-go).
 *
 * [iso] = true draws the same world in a 3D isometric view (the "Real-Time
 * Mapping" look). Only the projection changes; all data stays in metres.
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
    iso: Boolean = false,
    grid: GridImage? = null,                   // Phase 3 occupancy layer
    targetPos: Pair<Double, Double>? = null,   // tracked lidar target
    warning: Boolean = false,                  // proximity buzzer active
    goal: Pair<Double, Double>? = null,        // Phase 4 go-to goal
    path: FloatArray = FloatArray(0),          // planned path (x,y pairs)
    onLongPress: ((Double, Double) -> Unit)? = null,   // world metres
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
            .transformable(state = transformState)
            .pointerInput(Unit) {
                // Long-press sets a go-to goal (Phase 4). Screen -> world
                // with the SAME transform the drawer uses (y flip included).
                detectTapGestures(onLongPress = { p ->
                    val cx = size.width / 2f + offset.x
                    val cy = size.height / 2f + offset.y
                    onLongPress?.invoke((p.x - cx) / scale.toDouble(),
                        (cy - p.y) / scale.toDouble())
                })
            },
    ) {
        val cx = size.width / 2f + offset.x
        val cy = size.height / 2f + offset.y

        fun toScreen(wx: Double, wy: Double): Offset =
            if (iso) isoPoint(wx, wy, cx, cy, scale)
            else Offset(cx + (wx * scale).toFloat(), cy - (wy * scale).toFloat())

        // Occupancy layer under everything (2D view; the iso projection
        // draws the same cells per-face later if needed).
        grid?.let { g ->
            if (!iso) {
                val tl = toScreen(g.worldX, g.worldY + g.sizeYM)
                val br = toScreen(g.worldX + g.sizeXM, g.worldY)
                val dw = (br.x - tl.x).toInt().coerceAtLeast(1)
                val dh = (tl.y - br.y).toInt().coerceAtLeast(1)
                drawImage(
                    g.image,
                    dstOffset = androidx.compose.ui.unit.IntOffset(
                        tl.x.toInt(), tl.y.toInt()),
                    dstSize = androidx.compose.ui.unit.IntSize(dw, dh),
                )
            }
        }

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

        // ---- Phase 4: planned path + goal marker ------------------------
        if (path.size >= 4) {
            val pathStroke = Color.White.copy(alpha = 0.75f)
            for (i in 2 until path.size step 2) {
                drawLine(
                    color = pathStroke,
                    start = toScreen(path[i - 2].toDouble(), path[i - 1].toDouble()),
                    end = toScreen(path[i].toDouble(), path[i + 1].toDouble()),
                    strokeWidth = 4f,
                )
            }
        }
        goal?.let { (gx, gy) ->
            val gp = toScreen(gx, gy)
            drawCircle(color = RobotOrange, radius = 12f, center = gp)
            drawCircle(color = Color.White, radius = 12f, center = gp,
                style = Stroke(width = 3f))
            drawLine(color = RobotOrange, start = Offset(gp.x - 18f, gp.y),
                end = Offset(gp.x + 18f, gp.y), strokeWidth = 3f)
            drawLine(color = RobotOrange, start = Offset(gp.x, gp.y - 18f),
                end = Offset(gp.x, gp.y + 18f), strokeWidth = 3f)
        }

        // ---- lidar behaviour overlays -----------------------------------
        // Tracked target (laser_Tracker selection): green marker + line of
        // sight from the robot.
        targetPos?.let { (tx, ty) ->
            val tp = toScreen(tx, ty)
            val rp0 = toScreen(robotX.toDouble(), robotY.toDouble())
            drawLine(
                color = Color(0xFF4CAF50).copy(alpha = 0.7f),
                start = rp0, end = tp, strokeWidth = 3f,
            )
            drawCircle(color = Color(0xFF4CAF50), radius = 10f, center = tp)
            drawCircle(color = Color.White, radius = 10f, center = tp,
                style = Stroke(width = 3f))
        }
        // Warning (laser_Warning buzzer): red frame around the viewport.
        if (warning) {
            val inset = 6f
            drawRect(
                color = DangerRed,
                topLeft = Offset(inset, inset),
                size = Size(size.width - 2 * inset, size.height - 2 * inset),
                style = Stroke(width = 8f),
            )
        }

        // Robot: body + heading arrow (x forward). In iso mode the body is a
        // short cylinder: shadow on the floor, raised top disc.
        val rp = toScreen(robotX.toDouble(), robotY.toDouble())
        val r = config.robotRadiusM.toFloat() * scale
        val lift = if (iso) 10f else 0f
        val top = Offset(rp.x, rp.y - lift)
        if (iso) {
            drawOval(
                color = Color.Black.copy(alpha = 0.35f),
                topLeft = Offset(rp.x - r, rp.y - r * 0.5f),
                size = Size(2 * r, r),
            )
            drawOval(
                color = Color(0xFFB34700),
                topLeft = Offset(top.x - r, top.y - r * 0.5f + 6f),
                size = Size(2 * r, r),
            )
        }
        drawOval(
            color = RobotOrange,
            topLeft = Offset(top.x - r, top.y - r * 0.5f),
            size = Size(2 * r, r),
        )

        val armM = config.robotRadiusM * 1.2
        val tip = toScreen(
            robotX + armM * cos(robotTheta),
            robotY + armM * sin(robotTheta),
        ).let { Offset(it.x, it.y - lift) }
        drawLine(color = Color.White, start = top, end = tip, strokeWidth = 6f)
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

/** Isometric projection of a world point (metres, y up) to screen pixels. */
private fun isoPoint(wx: Double, wy: Double, cx: Float, cy: Float, s: Float): Offset {
    val x = wx.toFloat()
    val y = wy.toFloat()
    return Offset(cx + (x - y) * COS30 * s, cy - (x + y) * 0.5f * s)
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
