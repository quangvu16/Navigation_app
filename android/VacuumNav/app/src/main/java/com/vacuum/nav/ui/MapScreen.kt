package com.vacuum.nav.ui

import com.vacuum.nav.ui.theme.Rd
import com.vacuum.nav.ui.theme.Sp
import com.vacuum.nav.ui.theme.Ts

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material3.Button
import androidx.compose.material3.Icon
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.FilledTonalButton
import androidx.compose.material.icons.filled.Block
import androidx.compose.material.icons.filled.Explore
import androidx.compose.material.icons.filled.Gamepad
import androidx.compose.material.icons.filled.GridOff
import androidx.compose.material.icons.filled.GridOn
import androidx.compose.material.icons.filled.ViewInAr
import androidx.compose.material.icons.filled.Save
import androidx.compose.material.icons.filled.Stop
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.graphics.asImageBitmap
import com.vacuum.nav.data.MapStore
import com.vacuum.nav.data.RunStats
import com.vacuum.nav.data.ZoneStore
import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.robot.Scan
import com.vacuum.nav.service.RobotService
import com.vacuum.nav.ui.theme.DangerRed
import com.vacuum.nav.ui.theme.RobotOrange
import kotlinx.coroutines.delay

/**
 * Map, in the shape of the reference "Real-Time Mapping" screen: live stats
 * header (Clean Area / Clean Times / Clean Time), the growing map, a right-
 * hand icon rail, Full/Rooms/Zones mode tabs and Start/Pause/Stop + E-STOP.
 */
@Composable
fun MapScreen(
    onOpenZones: () -> Unit,
    onOpenDrive: () -> Unit,
    modifier: Modifier = Modifier,
) {
    val link = RobotService.link
    val telemetry by (link?.telemetry
        ?: remember { kotlinx.coroutines.flow.MutableStateFlow(
            com.vacuum.nav.robot.Telemetry()) }).collectAsState()
    val scan = remember { mutableStateOf<Scan?>(null) }
    val config = remember { RobotConfig() }
    LaunchedEffect(link) { link?.scans?.collect { scan.value = it } }

    val ctx = androidx.compose.ui.platform.LocalContext.current
    val zones = remember { mutableStateOf(ZoneStore.load(ctx)) }

    var mode by remember { mutableIntStateOf(0) }   // 0 Full, 1 Rooms, 2 Zones
    var showGrid by remember { mutableStateOf(true) }
    var iso by remember { mutableStateOf(true) }   // 3D isometric view
    val autoClean = RobotService.autoClean
    val eStop = RobotService.eStop

    // Tick the stat clock while cleaning (RunStats integrates pose on the
    // service thread; this only refreshes the displayed seconds).
    var tick by remember { mutableIntStateOf(0) }
    LaunchedEffect(autoClean) {
        while (autoClean) {
            delay(1000)
            tick++
        }
    }

    Column(modifier.fillMaxSize()) {
        // ---- stats header ------------------------------------------------
        Row(
            Modifier.fillMaxWidth().padding(horizontal = Sp.Md, vertical = Sp.Xs),
            horizontalArrangement = Arrangement.SpaceBetween,
        ) {
            Stat("Clean Area", RunStats.areaText())
            Stat("Clean Times", "${RunStats.runs}")
            Stat("Clean Time", RunStats.timeText())
        }

        // ---- map + right icon rail --------------------------------------
        val lidarState by RobotService.lidarCtl.state.collectAsState()
        // Phase 3: occupancy layer, bitmap rebuilt only on version change.
        val mapSnap by (RobotService.mapper?.snapshot
            ?: remember { kotlinx.coroutines.flow.MutableStateFlow(null) })
            .collectAsState()
        val gridImage = remember(mapSnap) { mapSnap?.let(::buildGridImage) }
        // Phase 4: go-to goal + planned path + state chip.
        val navState by (RobotService.nav?.state
            ?: remember { kotlinx.coroutines.flow.MutableStateFlow(
                com.vacuum.nav.robot.NavUiState()) }).collectAsState()
        var navMessage by remember { mutableStateOf<String?>(null) }
        var saveHint by remember { mutableStateOf<String?>(null) }
        val ctx = androidx.compose.ui.platform.LocalContext.current
        // Phase 7: Rooms tab runs segmentation (cached in C++) and tints
        // the room cells over the occupancy layer.
        val segResult = remember(mapSnap?.version, mode, RobotService.nav?.mergeVersion) {
            if (mode == 1) RobotService.nav?.segment() else null
        }
        val roomsImage = remember(mapSnap?.version, mode,
            RobotService.nav?.mergeVersion) {
            val snap = mapSnap
            if (mode == 1 && snap != null)
                RobotService.nav?.segment()?.let { buildRoomsImage(snap, it) }
            else null
        }
        Box(Modifier.weight(1f).fillMaxWidth()) {
            // Live mapper output wins; a loaded floor map (MapStore) is the
            // base layer until the mapper has built something of its own.
            val baseImage = remember(mapSnap, MapStore.current) {
                (mapSnap ?: MapStore.current)?.let(::buildGridImage)
            }
            MapCanvas(
                config = config,
                scan = scan.value,
                robotX = telemetry.pose.x,
                robotY = telemetry.pose.y,
                robotTheta = telemetry.pose.theta,
                sonar = floatArrayOf(
                    telemetry.sonarFrontM, telemetry.sonarLeftM,
                    telemetry.sonarRightM,
                ),
                zones = zones.value,
                showGrid = showGrid,
                iso = iso,
                grid = roomsImage ?: gridImage ?: baseImage,
                targetPos = lidarState.target?.let {
                    RobotService.lidarCtl.targetWorld(
                        telemetry.pose.x.toDouble(),
                        telemetry.pose.y.toDouble(),
                        telemetry.pose.theta.toDouble(),
                    )
                },
                warning = lidarState.warning.buzzer,
                goal = navState.goal,
                path = navState.path,
                onLongPress = { wx, wy ->
                    val ok = RobotService.startGoTo(ctx, wx, wy)
                    navMessage = if (ok) null else "Invalid goal (obstacle)"
                },
                modifier = Modifier.fillMaxSize(),
            )
            Column(
                Modifier.align(Alignment.CenterEnd).padding(Sp.Xs),
                verticalArrangement = Arrangement.spacedBy(10.dp),
            ) {
                RailButton(Icons.Filled.Block, "Zones") { onOpenZones() }
                RailButton(Icons.Filled.Gamepad, "Drive") { onOpenDrive() }
                RailButton(Icons.Filled.Save, "Save") {
                    val snap = mapSnap
                    if (snap != null) {
                        val n = MapStore.list(ctx)
                            .maxOfOrNull { it.floor } ?: 0
                        MapStore.save(ctx, "Floor ${n + 1}", n + 1, snap)
                        saveHint = "Saved Floor ${n + 1}"
                    } else {
                        saveHint = "Map a bit first"
                    }
                }
                RailButton(
                    if (RobotService.exploreActive) Icons.Filled.Stop else Icons.Filled.Explore,
                    if (RobotService.exploreActive) "Stop" else "Explore",
                ) { RobotService.toggleExplore() }
                RailButton(if (showGrid) Icons.Filled.GridOn else Icons.Filled.GridOff, "Grid") {
                    showGrid = !showGrid
                }
                RailButton(Icons.Filled.ViewInAr, if (iso) "3D" else "2D") { iso = !iso }
            }
            if (mode == 1) {
                val rooms = segResult?.rooms
                if (rooms.isNullOrEmpty()) {
                    ModeHint("No rooms yet — map more of the space first")
                } else {
                    // Tap to select rooms for a targeted clean (§5/§8);
                    // stable ids carry user names (RoomNames).
                    var renaming by remember { mutableStateOf<Int?>(null) }
                    var roomName by remember { mutableStateOf("") }
                    Row(
                        Modifier.fillMaxWidth().padding(horizontal = 4.dp),
                        horizontalArrangement = Arrangement.spacedBy(Sp.Xxs),
                    ) {
                        rooms.forEach { r ->
                            val selected =
                                r.id in RobotService.selectedRooms
                            Surface(
                                onClick = {
                                    RobotService.selectedRooms =
                                        if (selected) RobotService
                                            .selectedRooms - r.id
                                        else RobotService
                                            .selectedRooms + r.id
                                },
                                shape = RoundedCornerShape(50),
                                color = if (selected) RobotOrange
                                else MaterialTheme.colorScheme.surfaceVariant,
                            ) {
                                Text(
                                    com.vacuum.nav.data.RoomNames
                                        .get(ctx, r.id) +
                                        " · %.1f m²".format(r.areaM2),
                                    fontSize = Ts.Caption,
                                    color = if (selected) Color.White
                                    else MaterialTheme.colorScheme
                                        .onSurfaceVariant,
                                    modifier = Modifier.padding(
                                        horizontal = 10.dp, vertical = Sp.Xs),
                                )
                            }
                        }
                    }
                    // Rename / merge (stable-id edits, prompt §5).
                    Row(
                        Modifier.fillMaxWidth().padding(horizontal = 4.dp),
                        horizontalArrangement = Arrangement.spacedBy(Sp.Xs),
                    ) {
                        if (RobotService.selectedRooms.size == 1) {
                            TextButton(onClick = {
                                val id = RobotService.selectedRooms.first()
                                roomName = com.vacuum.nav.data.RoomNames
                                    .get(ctx, id)
                                renaming = id
                            }) { Text("Rename", fontSize = Ts.Caption) }
                        }
                        if (RobotService.selectedRooms.size == 2) {
                            TextButton(onClick = {
                                val (a, b) = RobotService.selectedRooms
                                    .toList()
                                RobotService.nav?.mergeRooms(a, b)
                                RobotService.selectedRooms = setOf(b)
                                navMessage = "Merged rooms"
                            }) { Text("Merge", fontSize = Ts.Caption) }
                        }
                        if (RobotService.selectedRooms.isNotEmpty()) {
                            TextButton(onClick = {
                                RobotService.selectedRooms = emptySet()
                            }) { Text("Clear", fontSize = Ts.Caption) }
                        }
                    }
                    if (renaming != null) {
                        androidx.compose.material3.AlertDialog(
                            onDismissRequest = { renaming = null },
                            title = { Text("Rename room") },
                            text = {
                                androidx.compose.material3.OutlinedTextField(
                                    value = roomName,
                                    onValueChange = { roomName = it },
                                    singleLine = true,
                                    label = { Text("Name") },
                                )
                            },
                            confirmButton = {
                                androidx.compose.material3.TextButton(
                                    onClick = {
                                        renaming?.let {
                                            com.vacuum.nav.data.RoomNames
                                                .set(ctx, it, roomName)
                                        }
                                        renaming = null
                                    },
                                ) { Text("Save") }
                            },
                            dismissButton = {
                                androidx.compose.material3.TextButton(
                                    onClick = { renaming = null },
                                ) { Text("Cancel") }
                            },
                        )
                    }
                }
            }
            if (mode == 2) ModeHint("Zones: draw on the Zones tab")
            // Phase 4: go-to status / rejection feedback.
            val navLabel = when {
                saveHint != null -> saveHint
                navMessage != null -> navMessage
                navState.state == com.vacuum.nav.robot.NavState.REJECTED ->
                    "Invalid goal (obstacle)"
                navState.active ->
                    com.vacuum.nav.robot.NavState.label(navState.state)
                navState.state == com.vacuum.nav.robot.NavState.REACHED ->
                    "Goal reached"
                navState.state == com.vacuum.nav.robot.NavState.STUCK ->
                    "Stuck — gave up"
                else -> null
            }
            if (navLabel != null) {
                ModeHint(navLabel + if (navState.active)
                    "  ·  long-press to re-goal, joystick to cancel" else "")
            } else if (RobotService.exploreActive) {
                ModeHint("Exploring… (frontier search, §6)")
            } else if (!navState.active && navState.goal == null) {
                ModeHint("Long-press the map to drive there")
            }
            // Clear one-shot messages once the state moves on.
            LaunchedEffect(navState.state) {
                if (navState.state == com.vacuum.nav.robot.NavState.REACHED ||
                    navState.state == com.vacuum.nav.robot.NavState.STUCK
                ) {
                    delay(4000)
                    navMessage = null
                    RobotService.cancelGoTo()
                }
            }
            LaunchedEffect(saveHint) {
                if (saveHint != null) {
                    delay(3000)
                    saveHint = null
                }
            }
        }

        // ---- mode tabs ---------------------------------------------------
        Row(
            Modifier.fillMaxWidth().padding(horizontal = Sp.Md),
            horizontalArrangement = Arrangement.spacedBy(Sp.Xs),
        ) {
            ModeTab("Full", mode == 0, Modifier.weight(1f)) { mode = 0 }
            ModeTab("Rooms", mode == 1, Modifier.weight(1f)) { mode = 1 }
            ModeTab("Zones", mode == 2, Modifier.weight(1f)) { mode = 2 }
        }

        // ---- transport controls -----------------------------------------
        Row(
            Modifier.fillMaxWidth().padding(Sp.Md),
            horizontalArrangement = Arrangement.spacedBy(Sp.Sm),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Button(
                onClick = {
                    if (eStop) RobotService.setEstop(false)
                    val running = autoClean ||
                        RobotService.coverageQueue.isNotEmpty()
                    if (running) {
                        RobotService.stopCleaning()
                    } else if (mode == 1) {
                        // Rooms tab: targeted boustrophedon clean (§8).
                        if (!RobotService.startRoomCleaning(ctx))
                            navMessage = "Select rooms first"
                    } else {
                        RobotService.startCleaning()
                    }
                },
                Modifier.weight(1f).height(48.dp),
                shape = RoundedCornerShape(50),
                colors = ButtonDefaults.buttonColors(
                    containerColor =
                        if (autoClean || RobotService.coverageQueue.isNotEmpty())
                            Color(0xFF9E9E9E)
                        else RobotOrange,
                ),
            ) {
                Text(
                    if (autoClean ||
                        RobotService.coverageQueue.isNotEmpty()
                    ) "Pause" else "Start",
                    fontWeight = FontWeight.SemiBold,
                )
            }
            FilledTonalButton(
                onClick = { RobotService.goDock(ctx) },
                Modifier.weight(1f).height(48.dp),
                shape = RoundedCornerShape(50),
            ) { Text("Dock") }
            Button(
                onClick = { RobotService.setEstop(!eStop) },
                Modifier.weight(1f).height(48.dp),
                shape = RoundedCornerShape(50),
                colors = ButtonDefaults.buttonColors(
                    containerColor = DangerRed,
                ),
            ) {
                Text(if (eStop) "RESET" else "E-STOP",
                    fontWeight = FontWeight.Bold)
            }
        }
        @Suppress("UNUSED_EXPRESSION")
        tick  // read so recomposition tracks the stat clock
    }
}

/**
 * Snapshot -> ImageBitmap: unknown transparent (dark bg shows through),
 * free light, occupied wall-dark. Row 0 of the snapshot is the WORLD
 * BOTTOM, so the bitmap rows are flipped here (screen row 0 = top).
 */
private fun buildGridImage(s: com.vacuum.nav.robot.GridSnapshot): GridImage {
    val px = IntArray(s.width * s.height)
    for (y in 0 until s.height) {
        val row = s.height - 1 - y
        for (x in 0 until s.width) {
            px[row * s.width + x] = when (s.cells[y * s.width + x].toInt()) {
                1 -> 0xFF4A4D52.toInt()   // occupied (theme MapWall)
                0 -> 0xFFE8E6E1.toInt()   // free (theme MapFree)
                else -> 0x00000000        // unknown: transparent
            }
        }
    }
    val bmp = android.graphics.Bitmap.createBitmap(
        px, s.width, s.height, android.graphics.Bitmap.Config.ARGB_8888)
    return GridImage(
        image = bmp.asImageBitmap(),
        worldX = s.originX * s.resolution.toDouble(),
        worldY = s.originY * s.resolution.toDouble(),
        sizeXM = s.width * s.resolution.toDouble(),
        sizeYM = s.height * s.resolution.toDouble(),
    )
}

/**
 * Occupancy walls + room tints in one bitmap (Phase 7): room cells get a
 * stable hue per id (id*67 deg on the wheel), walls keep the dark wall
 * colour, unknown stays transparent. Row 0 = world bottom (flipped here).
 */
private fun buildRoomsImage(
    snap: com.vacuum.nav.robot.GridSnapshot,
    seg: com.vacuum.nav.robot.NavEngine.SegmentResult,
): GridImage {
    val px = IntArray(snap.width * snap.height)
    // Occupancy first (walls visible under/between rooms).
    for (y in 0 until snap.height) {
        val row = snap.height - 1 - y
        for (x in 0 until snap.width) {
            px[row * snap.width + x] = when (snap.cells[y * snap.width + x].toInt()) {
                1 -> 0xFF4A4D52.toInt()
                0 -> 0xFFE8E6E1.toInt()
                else -> 0x00000000
            }
        }
    }
    // Overlay room colours (same flip).
    if (seg.labels.size == snap.width * snap.height) {
        for (y in 0 until snap.height) {
            val row = snap.height - 1 - y
            for (x in 0 until snap.width) {
                val id = seg.labels[y * snap.width + x].toInt() and 0xFF
                if (id > 0) {
                    val hue = ((id * 67) % 360).toFloat()
                    px[row * snap.width + x] =
                        android.graphics.Color.HSVToColor(
                            170, floatArrayOf(hue, 0.55f, 0.95f))
                }
            }
        }
    }
    val bmp = android.graphics.Bitmap.createBitmap(
        px, snap.width, snap.height, android.graphics.Bitmap.Config.ARGB_8888)
    return GridImage(
        image = bmp.asImageBitmap(),
        worldX = snap.originX * snap.resolution.toDouble(),
        worldY = snap.originY * snap.resolution.toDouble(),
        sizeXM = snap.width * snap.resolution.toDouble(),
        sizeYM = snap.height * snap.resolution.toDouble(),
    )
}

@Composable
private fun Stat(label: String, value: String) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(value, fontSize = Ts.Headline, fontWeight = FontWeight.Bold)
        Text(label, fontSize = Ts.Caption,
            color = MaterialTheme.colorScheme.onSurfaceVariant)
    }
}

@Composable
private fun RailButton(icon: ImageVector, desc: String, onClick: () -> Unit) {
    Surface(
        onClick = onClick,
        shape = CircleShape,
        color = MaterialTheme.colorScheme.surface.copy(alpha = 0.92f),
        shadowElevation = 4.dp,
    ) {
        Column(
            Modifier.size(52.dp).padding(6.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.Center,
        ) {
            Icon(icon, contentDescription = desc, modifier = Modifier.size(22.dp))
            Text(desc, fontSize = Ts.Micro,
                color = MaterialTheme.colorScheme.onSurfaceVariant)
        }
    }
}

@Composable
private fun ModeTab(label: String, selected: Boolean, modifier: Modifier,
                    onClick: () -> Unit) {
    Surface(
        onClick = onClick,
        modifier = modifier.height(40.dp),
        shape = RoundedCornerShape(50),
        color = if (selected) RobotOrange
        else MaterialTheme.colorScheme.surfaceVariant,
    ) {
        Box(contentAlignment = Alignment.Center) {
            Text(
                label, fontSize = Ts.Body,
                fontWeight = if (selected) FontWeight.Bold else FontWeight.Normal,
                color = if (selected) Color.White
                else MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
    }
}

@Composable
private fun androidx.compose.foundation.layout.BoxScope.ModeHint(text: String) {
    Surface(
        Modifier.align(Alignment.BottomCenter).padding(bottom = 16.dp),
        shape = RoundedCornerShape(50),
        color = Color.Black.copy(alpha = 0.6f),
    ) {
        Text(
            text, color = Color.White, fontSize = Ts.Caption,
            modifier = Modifier.padding(horizontal = 14.dp, vertical = 8.dp),
        )
    }
}
