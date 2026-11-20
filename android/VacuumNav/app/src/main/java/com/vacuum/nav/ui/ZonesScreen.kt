package com.vacuum.nav.ui

import com.vacuum.nav.ui.theme.Rd
import com.vacuum.nav.ui.theme.Sp
import com.vacuum.nav.ui.theme.Ts

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
import androidx.compose.material3.Surface
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.vacuum.nav.data.NoGoZone
import com.vacuum.nav.data.ZoneStore
import com.vacuum.nav.ui.theme.DangerRed
import com.vacuum.nav.ui.theme.RobotOrange
import kotlin.math.abs
import kotlin.math.max

private val HandleBlue = Color(0xFF2196F3)
private const val COS30 = 0.8660254f
private const val ISO_SCALE = 60f      // px per metre in the 3D editor
private const val HANDLE_PX = 36f      // generous handle hit target

/** World metres (x right, y up) -> isometric screen pixels around (cx, cy). */
private fun isoToScreen(wx: Double, wy: Double, cx: Float, cy: Float): Offset =
    Offset(
        cx + ((wx - wy) * COS30 * ISO_SCALE).toFloat(),
        cy - ((wx + wy) * 0.5 * ISO_SCALE).toFloat(),
    )

/** Inverse of [isoToScreen]: a touch on the floor -> world metres. */
private fun isoToWorld(sx: Float, sy: Float, cx: Float, cy: Float): Pair<Double, Double> {
    val a = ((sx - cx) / (COS30 * ISO_SCALE)).toDouble()   // x - y
    val b = ((cy - sy) / (0.5f * ISO_SCALE)).toDouble()    // x + y
    return Pair((a + b) / 2.0, (b - a) / 2.0)
}

/** Corner k of a zone: 0 TL, 1 TR, 2 BR, 3 BL (world metres). */
private fun NoGoZone.corner(k: Int): Pair<Double, Double> = when (k) {
    0 -> Pair(x, y + h)
    1 -> Pair(x + w, y + h)
    2 -> Pair(x + w, y)
    else -> Pair(x, y)
}

private fun NoGoZone.containsWorld(p: Pair<Double, Double>): Boolean =
    p.first >= x && p.first <= x + w && p.second >= y && p.second <= y + h

/** A flat rectangle on the floor, projected to an isometric quad. */
private fun quadPath(x1: Double, y1: Double, x2: Double, y2: Double,
                     cx: Float, cy: Float): Path {
    val pts = listOf(
        isoToScreen(x1, y1, cx, cy),
        isoToScreen(x2, y1, cx, cy),
        isoToScreen(x2, y2, cx, cy),
        isoToScreen(x1, y2, cx, cy),
    )
    return Path().apply {
        moveTo(pts[0].x, pts[0].y)
        for (p in pts.drop(1)) lineTo(p.x, p.y)
        close()
    }
}

/**
 * No-Go Zones editor, now in the 3D isometric view of the redesign. Drawing
 * and touch both go through isoToScreen / isoToWorld, so every edit is still
 * made in world metres. Drag the body to move, a corner to resize.
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

    Column(modifier.fillMaxSize().padding(Sp.Md)) {
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text("No-Go Zones", fontSize = Ts.Display, fontWeight = FontWeight.Bold)
            Text(
                "${zones.size} zone${if (zones.size == 1) "" else "s"}",
                fontSize = Ts.Body,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
        Spacer(Modifier.height(4.dp))
        Text(
            "Red blocks on the 3D map are off-limits. Drag the body to move, " +
                "a corner to resize.",
            fontSize = Ts.Caption,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
        Spacer(Modifier.height(12.dp))

        // ---- the 3D editor canvas ----------------------------------------
        Canvas(
            Modifier.weight(1f).fillMaxWidth()
                .clip(RoundedCornerShape(Rd.L))
                .background(MaterialTheme.colorScheme.surface)
                .pointerInput(zones.size) {
                    val cx = size.width / 2f
                    val cy = size.height / 2f
                    var mode = 0      // 0 none, 1 move, 2..5 resize corners
                    var zoneIdx = -1
                    var lastWorld = Pair(0.0, 0.0)

                    detectDragGestures(
                        onDragStart = { start ->
                            val w0 = isoToWorld(start.x, start.y, cx, cy)
                            lastWorld = w0
                            mode = 0
                            zoneIdx = -1
                            var found = false
                            // Handles of the selected zone first, then its
                            // body, then any other zone, then empty floor.
                            val sel = zones.indexOfFirst { it.id == selectedId }
                            if (sel >= 0) {
                                val z = zones[sel]
                                for (k in 0..3) {
                                    val c = z.corner(k)
                                    val s = isoToScreen(c.first, c.second, cx, cy)
                                    if (abs(s.x - start.x) < HANDLE_PX &&
                                        abs(s.y - start.y) < HANDLE_PX
                                    ) {
                                        mode = 2 + k
                                        zoneIdx = sel
                                        found = true
                                        break
                                    }
                                }
                                if (!found && z.containsWorld(w0)) {
                                    mode = 1
                                    zoneIdx = sel
                                    found = true
                                }
                            }
                            if (!found) {
                                zones.forEachIndexed { i, z ->
                                    if (!found && z.containsWorld(w0)) {
                                        selectedId = z.id
                                        zoneIdx = i
                                        mode = 1
                                        found = true
                                    }
                                }
                            }
                            if (!found) selectedId = null
                        },
                        onDrag = { change, _ ->
                            if (mode == 0 || zoneIdx < 0) return@detectDragGestures
                            change.consume()
                            val w = isoToWorld(change.position.x, change.position.y, cx, cy)
                            val dx = w.first - lastWorld.first
                            val dy = w.second - lastWorld.second
                            lastWorld = w
                            val z = zones[zoneIdx]
                            // A corner drag moves two edges; the opposite
                            // corner stays fixed (world y is up).
                            zones[zoneIdx] = when (mode) {
                                1 -> z.copy(x = z.x + dx, y = z.y + dy)
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

            // Floor grid + the room (light reference-style background).
            drawRect(Color(0xFFF4F6F8))
            val gridColor = Color(0xFFE2E6EA)
            for (i in -4..8) {
                drawLine(gridColor,
                    isoToScreen(i * 0.5, -4.0, cx, cy),
                    isoToScreen(i * 0.5, 8.0, cx, cy))
                drawLine(gridColor,
                    isoToScreen(-4.0, i * 0.5, cx, cy),
                    isoToScreen(8.0, i * 0.5, cx, cy))
            }
            drawPath(quadPath(0.1, 0.1, 3.9, 3.9, cx, cy), Color(0xFFB3E5FC))

            zones.forEach { z ->
                val p = quadPath(z.x, z.y, z.x + z.w, z.y + z.h, cx, cy)
                val on = z.enabled
                drawPath(p, if (on) DangerRed.copy(alpha = 0.4f) else Color(0x66888888))
                drawPath(p, if (on) DangerRed else Color.Gray, style = Stroke(3f))
                if (z.id == selectedId) {
                    for (k in 0..3) {
                        val c = z.corner(k)
                        val s = isoToScreen(c.first, c.second, cx, cy)
                        drawCircle(HandleBlue, radius = 14f, center = s)
                        drawCircle(Color.White, radius = 14f, center = s,
                            style = Stroke(3f))
                    }
                }
            }
        }

        // ---- per-zone list: name, size, on/off switch (design) ----------
        zones.forEach { z ->
            Spacer(Modifier.height(8.dp))
            Surface(
                onClick = { selectedId = z.id },
                modifier = Modifier.fillMaxWidth(),
                shape = RoundedCornerShape(Rd.M),
                color = if (z.id == selectedId) MaterialTheme.colorScheme.primaryContainer
                else MaterialTheme.colorScheme.surface,
            ) {
                Row(
                    Modifier.fillMaxWidth().padding(horizontal = Sp.Md, vertical = 8.dp),
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    Column(Modifier.weight(1f)) {
                        Text(z.name.ifBlank { "Zone" }, fontSize = Ts.Body,
                            fontWeight = FontWeight.Medium)
                        Text("%.1f × %.1f m".format(z.w, z.h), fontSize = Ts.Caption,
                            color = MaterialTheme.colorScheme.onSurfaceVariant)
                    }
                    Switch(
                        checked = z.enabled,
                        onCheckedChange = { on ->
                            val i = zones.indexOfFirst { it.id == z.id }
                            if (i >= 0) {
                                zones[i] = zones[i].copy(enabled = on)
                                dirty = true
                            }
                        },
                    )
                }
            }
        }

        Spacer(Modifier.height(12.dp))
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(Sp.Sm),
        ) {
            OutlinedButton(
                onClick = {
                    zones.add(NoGoZone(nextId++, -0.6, -0.6, 1.2, 1.2, name = "Zone ${zones.size + 1}"))
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
                    containerColor = RobotOrange,
                ),
            ) { Text("Save", fontWeight = FontWeight.Bold) }
        }
    }
}
