package com.vacuum.nav.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.rememberTransformableState
import androidx.compose.foundation.gestures.transformable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.produceState
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.nativeCanvas
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.robot.RobotLink
import com.vacuum.nav.robot.Scan
import com.vacuum.nav.service.RobotService
import com.vacuum.nav.ui.theme.DangerRed
import com.vacuum.nav.ui.theme.LidarCyan
import com.vacuum.nav.ui.theme.MapFree
import com.vacuum.nav.ui.theme.MapUnknown
import com.vacuum.nav.ui.theme.RobotOrange
import com.vacuum.nav.ui.theme.VacuumTheme
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlin.math.cos
import kotlin.math.sin

/** Phase-1 home: live sim map + joystick + status (prompt §8 Phase 1). */
@Composable
fun HomeScreen() {
    // The brain lives in RobotService; wait for it rather than racing it.
    val link by produceState<RobotLink?>(initialValue = RobotService.link) {
        while (value == null) {
            delay(50)
            value = RobotService.link
        }
    }

    Scaffold { padding ->
        if (link == null) {
            Box(Modifier.fillMaxSize().padding(padding), contentAlignment = Alignment.Center) {
                Text("Starting robot brain…")
            }
            return@Scaffold
        }
        HomeContent(link!!, Modifier.padding(padding))
    }
}

@Composable
private fun HomeContent(link: RobotLink, modifier: Modifier = Modifier) {
    val config = remember { RobotConfig() }
    val telemetry by link.telemetry.collectAsState()
    val scan = remember { mutableStateOf<Scan?>(null) }

    LaunchedEffect(link) {
        link.scans.collect { scan.value = it }
    }

    // E-stop latch: while set, every command path forces zero (prompt §7).
    var eStop by remember { mutableStateOf(false) }
    val commandFlow = remember { MutableStateFlow(Pair(0f, 0f)) }

    LaunchedEffect(eStop) {
        commandFlow.collect { (drive, turn) ->
            if (eStop) {
                link.setWheelRpm(0f, 0f)
            } else {
                val (l, r) = config.velocityToWheelRpm(
                    drive * config.maxLinearMps,
                    turn * config.maxAngularRps,
                )
                link.setWheelRpm(l.toFloat(), r.toFloat())
            }
        }
    }

    // FPS meter driven by the frame clock (the acceptance metric: 60 fps).
    var fps by remember { mutableFloatStateOf(0f) }
    LaunchedEffect(Unit) {
        var frames = 0
        var lastMs = 0L
        while (true) {
            androidx.compose.runtime.withFrameNanos { now ->
                frames++
                val ms = now / 1_000_000
                if (lastMs == 0L) lastMs = ms
                if (ms - lastMs >= 500) {
                    fps = frames * 1000f / (ms - lastMs)
                    frames = 0
                    lastMs = ms
                }
            }
        }
    }

    Column(modifier.fillMaxSize()) {
        StatusRow(fps = fps, batteryV = telemetry.batteryV, eStop = eStop,
            state = if (eStop) "E-STOP" else "SIM")

        Box(Modifier.weight(1f).fillMaxWidth()) {
            MapCanvas(
                config = config,
                scan = scan.value,
                robotX = telemetry.pose.x,
                robotY = telemetry.pose.y,
                robotTheta = telemetry.pose.theta,
                sonar = floatArrayOf(
                    telemetry.sonarFrontM, telemetry.sonarLeftM, telemetry.sonarRightM
                ),
                modifier = Modifier.fillMaxSize(),
            )
        }

        Row(
            Modifier.fillMaxWidth().padding(16.dp),
            horizontalArrangement = Arrangement.SpaceEvenly,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Joystick(
                onDrive = { drive, turn -> commandFlow.value = Pair(drive, turn) },
            )
            Column(horizontalAlignment = Alignment.CenterHorizontally) {
                Button(
                    onClick = { eStop = false; commandFlow.value = Pair(0f, 0f) },
                    shape = CircleShape,
                    colors = ButtonDefaults.buttonColors(
                        containerColor = if (eStop) Color.Gray else RobotOrange
                    ),
                ) { Text("STOP") }
                Spacer(Modifier.height(12.dp))
                Button(
                    onClick = {
                        eStop = !eStop
                        link.setWheelRpm(0f, 0f)
                    },
                    shape = CircleShape,
                    colors = ButtonDefaults.buttonColors(containerColor = DangerRed),
                ) { Text(if (eStop) "RESET" else "E-STOP") }
            }
        }
    }
}

@Composable
private fun StatusRow(fps: Float, batteryV: Float, eStop: Boolean, state: String) {
    Row(
        Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 8.dp),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        Chip("$state", if (eStop) DangerRed else RobotOrange)
        Chip("⚡ ${"%.1f".format(batteryV)} V", Color(0xFF2E7D32))
        Chip("${"%.0f".format(fps)} fps", Color(0xFF1565C0))
    }
}

@Composable
private fun Chip(text: String, color: Color) {
    Surface(shape = RoundedCornerShape(50), color = color.copy(alpha = 0.15f)) {
        Text(
            text,
            color = color,
            fontSize = 13.sp,
            modifier = Modifier.padding(horizontal = 12.dp, vertical = 6.dp),
        )
    }
}

/**
 * The live map. World coordinates are metres (x right, y up); screen y is
 * down, so every draw flips Y (prompt §6 — goals and overlays always travel
 * in world metres, never pixels). One Matrix-free transform: pan/zoom state
 * applied uniformly. 60 fps target: one Canvas node, no allocations in the
 * per-point loop beyond the scan copy we already hold.
 */
@Composable
private fun MapCanvas(
    config: RobotConfig,
    scan: Scan?,
    robotX: Float,
    robotY: Float,
    robotTheta: Float,
    sonar: FloatArray,
    modifier: Modifier = Modifier,
) {
    var scale by remember { mutableFloatStateOf(90f) }     // px per metre
    var offset by remember { mutableStateOf(Offset.Zero) } // pan, px

    val transformState = rememberTransformableState { zoomChange, panChange, _ ->
        scale = (scale * zoomChange).coerceIn(20f, 400f)
        offset += panChange
    }

    Canvas(
        modifier = modifier
            .background(MapUnknown)
            .transformable(state = transformState),
    ) {
        // World (0,0) centred in the view + pan/zoom; Y flipped here only.
        val cx = size.width / 2f + offset.x
        val cy = size.height / 2f + offset.y

        fun toScreen(wx: Double, wy: Double): Offset =
            Offset(cx + (wx * scale).toFloat(), cy - (wy * scale).toFloat())

        // Floor plane: a subtle grid in world metres (the sim room is 4x4 m).
        drawGrid(::toScreen)

        // Lidar points: raw bins -> world via the same transform the C++
        // tests verify (scan_transform.h), then world -> screen.
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

        // Sonar arcs (front/left/right) as translucent wedges.
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
        drawLine(
            color = Color.White,
            start = rp,
            end = tip,
            strokeWidth = 6f,
        )
        val path = Path().apply {
            moveTo(tip.x, tip.y)
            lineTo(tip.x - 14f * cos(robotTheta - 0.5f), tip.y + 14f * sin(robotTheta - 0.5f))
            lineTo(tip.x - 14f * cos(robotTheta + 0.5f), tip.y + 14f * sin(robotTheta + 0.5f))
            close()
        }
        drawPath(path, Color.White)
    }
}

private fun DrawScope.drawGrid(toScreen: (Double, Double) -> Offset) {
    val gridColor = Color.White.copy(alpha = 0.08f)
    for (i in -4..8) {
        val a = toScreen(i * 0.5, -4.0)
        val b = toScreen(i * 0.5, 8.0)
        drawLine(gridColor, a, b, 1f)
        val c = toScreen(-4.0, i * 0.5)
        val d = toScreen(8.0, i * 0.5)
        drawLine(gridColor, c, d, 1f)
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
    // Base-frame angles of front / left / right HC-SR04s (x fwd, y left).
    val angles = doubleArrayOf(0.0, Math.PI / 2, -Math.PI / 2)
    for (i in 0..2) {
        val range = sonar[i]
        if (range <= 0f) continue
        val start = toScreen(robotX.toDouble(), robotY.toDouble())
        val mid = robotTheta + angles[i]  // screen angle: y flipped below
        val end = toScreen(
            robotX + range * cos(mid),
            robotY + range * sin(mid),
        )
        drawLine(
            color = DangerRed.copy(alpha = 0.4f),
            start = start,
            end = end,
            strokeWidth = 10f,
        )
    }
}
