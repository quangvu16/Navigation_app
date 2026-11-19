package com.vacuum.nav.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.size
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.unit.dp
import com.vacuum.nav.ui.theme.RobotOrange
import kotlin.math.hypot
import kotlin.math.min

/**
 * Virtual joystick with a deadman: output is (drive, turn) in [-1, 1]² while
 * the knob is held, and (0, 0) the moment it is released — the robot never
 * keeps rolling after the finger lifts (prompt §6).
 */
@Composable
fun Joystick(
    onDrive: (drive: Float, turn: Float) -> Unit,
    modifier: Modifier = Modifier,
    activeColor: Color = RobotOrange,
) {
    var knob by remember { mutableStateOf(Offset.Zero) }
    var held by remember { mutableStateOf(false) }

    Canvas(
        modifier = modifier
            .size(180.dp)
            .pointerInput(Unit) {
                detectDragGestures(
                    onDragStart = { held = true },
                    onDragEnd = {
                        held = false
                        knob = Offset.Zero
                        onDrive(0f, 0f)   // deadman release = stop
                    },
                    onDragCancel = {
                        held = false
                        knob = Offset.Zero
                        onDrive(0f, 0f)
                    },
                    onDrag = { change, dragAmount ->
                        change.consume()
                        val radius = min(size.width, size.height) / 2f * 0.85f
                        val next = knob + dragAmount
                        val len = hypot(next.x, next.y)
                        knob = if (len > radius) next * (radius / len) else next
                        // y is inverted: screen down = robot backward.
                        onDrive(-knob.y / radius, -knob.x / radius)
                    },
                )
            },
    ) {
        val c = Offset(size.width / 2f, size.height / 2f)
        val radius = min(size.width, size.height) / 2f * 0.85f
        drawCircle(
            color = Color.White.copy(alpha = 0.12f),
            radius = radius,
            center = c,
        )
        drawCircle(
            color = if (held) activeColor else Color.White.copy(alpha = 0.35f),
            radius = radius,
            center = c,
            style = Stroke(width = 4f),
        )
        drawCircle(
            color = if (held) activeColor else Color.White.copy(alpha = 0.5f),
            radius = radius / 3f,
            center = c + knob,
        )
    }
}
