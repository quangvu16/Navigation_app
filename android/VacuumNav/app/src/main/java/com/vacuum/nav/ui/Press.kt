package com.vacuum.nav.ui

import androidx.compose.animation.core.Spring
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.animation.core.spring
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.composed
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.input.pointer.pointerInput

/**
 * Phase 4 of the redesign (press feedback): compress-on-press scale for
 * primary CTAs. OBSERVES the gesture without consuming it, so the
 * Button/Surface underneath keeps its own click handling and ripple - we
 * only add the scale that a ripple alone doesn't communicate on large
 * targets. Built on the primitive awaitPointerEvent() loop (no gesture
 * helpers that would claim the stream).
 */
fun Modifier.pressScale(target: Float = 0.97f): Modifier = composed {
    var pressed by remember { mutableStateOf(false) }
    val scale by animateFloatAsState(
        targetValue = if (pressed) target else 1f,
        animationSpec = spring(
            dampingRatio = Spring.DampingRatioMediumBouncy,
            stiffness = Spring.StiffnessMedium,
        ),
        label = "pressScale",
    )
    this
        .graphicsLayer {
            scaleX = scale
            scaleY = scale
        }
        .pointerInput(Unit) {
            awaitPointerEventScope {
                while (true) {
                    // Down (any change pressed on this component) ...
                    var event = awaitPointerEvent()
                    while (event.changes.none { it.pressed }) {
                        event = awaitPointerEvent()
                    }
                    pressed = true
                    // ... until release or cancellation (child consumed it).
                    do {
                        event = awaitPointerEvent()
                    } while (event.changes.any { it.pressed })
                    pressed = false
                }
            }
        }
}
