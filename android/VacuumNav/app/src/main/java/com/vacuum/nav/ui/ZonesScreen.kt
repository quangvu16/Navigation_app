package com.vacuum.nav.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.vacuum.nav.data.NoGoZone
import com.vacuum.nav.data.ZoneStore
import com.vacuum.nav.ui.theme.DangerRed
import kotlin.math.abs
import kotlin.math.max

private val HandleBlue = Color(0xFF2196F3)

/**
 * No-Go Zones editor, in the shape of the reference screen: fixed map view,
 * a red rectangle with draggable blue corner handles (move by dragging the
 * body, resize by a corner). Coordinates are world metres end to end.
 */
@Composable
fun ZonesScreen(modifier: Modifier = Modifier) {
    val ctx = LocalContext.current
    val zones = remember { mutableStateListOf<NoGoZone>().apply {
        addAll(ZoneStore.load(ctx))
    } }
    var selectedId by remember { mutableStateOf<Long?>(null) }
    var nextId by remember { mutableStateOf(1000L) }
    var dirty by remember { mutableStateOf(false) }

    // Fixed view transform (no pan/zoom: gesture space belongs to editing).
    val scale = 90f

    Column(modifier.fillMaxSize().padding(16.dp)) {
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text("No-Go Zones", fontSize = 22.sp, fontWeight = FontWeight.Bold)
            Text(
                "${zones.size} zone${if (zones.size == 1) "" else "s"}",
                fontSize = 13.sp,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
        Spacer(Modifier.height(4.dp))
        Text(
            "Drag the body to move, a corner to resize. The robot will not " +
                "enter saved zones.",
            fontSize = 12.sp,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
        Spacer(Modifier.height(12.dp))

        // ---- the editor canvas ------------------------------------------
        Canvas(
            Modifier.weight(1f).fillMaxWidth()
                .clip(RoundedCornerShape(24.dp))
                .background(MaterialTheme.colorScheme.surface)
                .pointerInput(zones.size) {
                    val cx = size.width / 2f
                    val cy = size.height / 2f
                    fun toScreen(wx: Double, wy: Double) = Offset(
                        cx + (wx * scale).toFloat(),
                        cy - (wy * scale).toFloat(),
                    )
                    fun toWorld(sx: Float, sy: Float) =
                        ((sx - cx) / scale).toDouble() to ((cy - sy) / scale).toDouble()

                    var mode = 0   // 0 none, 1 move, 2..5 resize corners
                    var zoneIdx = -1
                    var lastWorld = Pair(0.0, 0.0)
                    val handlePx = 36f   // generous hit target

                    detectDragGestures(
                        onDragStart = { start ->
                            lastWorld = toWorld(start.x, start.y)
                            // Hit-test: handles of the selected zone first,
                            // then any zone body, then empty space.
                            val sel = zones.indexOfFirst { it.id == selectedId }
                            var found = false
                            if (sel >= 0) {
                                val z = zones[sel]
                                val corners = listOf(
                                    Triple(z.x, z.y + z.h, 1),          // TL
                                    Triple(z.x + z.w, z.y + z.h, 2),    // TR
                                    Triple(z.x + z.w, z.y, 3),          // BR
                                    Triple(z.x, z.y, 4),                // BL
                                )
                                for ((cxw, cyw, corner) in corners) {
                                    val s = toScreen(cxw, cyw)
                                    if (abs(s.x - start.x) < handlePx &&
                                        abs(s.y - start.y) < handlePx
                                    ) {
                                        mode = 1 + corner
                                        zoneIdx = sel
                                        found = true
                                        break
                                    }
                                }
                                if (!found) {
                                    val tl = toScreen(z.x, z.y + z.h)
                                    val br = toScreen(z.x + z.w, z.y)
                                    if (start.x in tl.x..br.x &&
                                        start.y in tl.y..br.y
                                    ) {
                                        mode = 1
                                        zoneIdx = sel
                                        found = true
                                    }
                                }
                            }
                            if (!found) {
                                // Tap a different zone to select it.
                                zones.forEachIndexed { i, z ->
                                    val tl = toScreen(z.x, z.y + z.h)
                                    val br = toScreen(z.x + z.w, z.y)
                                    if (start.x in tl.x..br.x &&
                                        start.y in tl.y..br.y
                                    ) {
                                        selectedId = z.id
                                        zoneIdx = i
                                        mode = 1
                                        found = true
                                    }
                                }
                            }
                            if (!found) {
                                mode = 0
                                zoneIdx = -1
                                selectedId = null
                            }
                        },
                        onDrag = { change, _ ->
                            if (mode == 0 || zoneIdx < 0) return@detectDragGestures
                            change.consume()
                            val w = toWorld(change.position.x, change.position.y)
                            val dx = w.first - lastWorld.first
                            val dy = w.second - lastWorld.second
                            lastWorld = w
                            val z = zones[zoneIdx]
                            // Dragging a corner moves TWO edges; the
                            // opposite corner stays fixed (world y is up).
                            zones[zoneIdx] = when (mode) {
                                1 -> z.copy(x = z.x + dx, y = z.y + dy)      // move
                                2 -> z.copy(                                  // TL
                                    x = z.x + dx, w = max(0.2, z.w - dx),
                                    h = max(0.2, z.h + dy))
                                3 -> z.copy(                                  // TR
                                    w = max(0.2, z.w + dx),
                                    h = max(0.2, z.h + dy))
                                4 -> z.copy(                                  // BR
                                    w = max(0.2, z.w + dx), y = z.y + dy,
                                    h = max(0.2, z.h - dy))
                                else -> z.copy(                               // BL
                                    x = z.x + dx, w = max(0.2, z.w - dx),
                                    y = z.y + dy, h = max(0.2, z.h - dy))
                            }
                            dirty = true
                        },
                    )
                },
        ) {
            val cx = size.width / 2f
            val cy = size.height / 2f
            fun toScreen(wx: Double, wy: Double) =
                Offset(cx + (wx * scale).toFloat(),
                    cy - (wy * scale).toFloat())

            // Grid + walls (light reference-style background).
            drawRect(Color(0xFFF4F6F8))
            for (i in -4..8) {
                val c = Color(0xFFE2E6EA)
                drawLine(c, toScreen(i * 0.5, -4.0), toScreen(i * 0.5, 8.0))
                drawLine(c, toScreen(-4.0, i * 0.5), toScreen(8.0, i * 0.5))
            }
            // The room outline (reference: coloured rooms; ours: neutral).
            val roomTl = toScreen(0.1, 3.9)
            val roomBr = toScreen(3.9, 0.1)
            drawRect(
                Color(0xFFB3E5FC),
                topLeft = roomTl,
                size = Size(roomBr.x - roomTl.x, roomBr.y - roomTl.y),
            )

            zones.forEach { z ->
                val tl = toScreen(z.x, z.y + z.h)
                val br = toScreen(z.x + z.w, z.y)
                val size = Size(br.x - tl.x, br.y - tl.y)
                drawRect(DangerRed.copy(alpha = 0.4f), topLeft = tl, size = size)
                drawRect(DangerRed, topLeft = tl, size = size,
                    style = androidx.compose.ui.graphics.drawscope.Stroke(3f))
                if (z.id == selectedId) {
                    listOf(
                        tl, Offset(br.x, tl.y), br, Offset(tl.x, br.y),
                    ).forEach { h ->
                        drawCircle(HandleBlue, radius = 14f, center = h)
                        drawCircle(Color.White, radius = 14f, center = h,
                            style = androidx.compose.ui.graphics.drawscope
                                .Stroke(3f))
                    }
                }
            }
        }

        Spacer(Modifier.height(12.dp))
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            OutlinedButton(
                onClick = {
                    zones.add(NoGoZone(nextId++, -0.6, -0.6, 1.2, 1.2))
                    selectedId = zones.last().id
                    dirty = true
                },
                Modifier.weight(1f),
            ) { Text("Add zone") }
            OutlinedButton(
                onClick = {
                    zones.clear()
                    selectedId = null
                    dirty = true
                },
                Modifier.weight(1f),
            ) { Text("Clear all") }
            Button(
                onClick = {
                    ZoneStore.save(ctx, zones.toList())
                    dirty = false
                },
                Modifier.weight(1f),
                enabled = dirty,
                colors = ButtonDefaults.buttonColors(
                    containerColor = Color(0xFF2BB3A0),
                    contentColor = Color(0xFF06201C),
                ),
            ) { Text("Save", fontWeight = FontWeight.Bold) }
        }
    }
}
