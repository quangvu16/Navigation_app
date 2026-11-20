package com.vacuum.nav.ui

import com.vacuum.nav.ui.theme.Rd
import com.vacuum.nav.ui.theme.Sp
import com.vacuum.nav.ui.theme.Ts

import android.content.ClipData
import android.content.ClipboardManager
import android.content.Context
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.Text
import androidx.compose.material3.TopAppBar
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.ArrowBack
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.vacuum.nav.data.RunStats
import com.vacuum.nav.robot.protocol.Packets
import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.service.RobotService

/**
 * Diagnostics (prompt §6): packet rates, ages, battery, sonar, map/nav
 * state, robot config, and a copyable text report (the "log export" for
 * now; file export lands with the share-sheet polish pass). Hardware
 * items (mirror test, wheel calibration wizard) light up when the real
 * link is connected.
 */
// TopAppBar is still marked experimental in this material3 build.
@OptIn(androidx.compose.material3.ExperimentalMaterial3Api::class)
@Composable
fun DiagnosticsScreen(onBack: () -> Unit, modifier: Modifier = Modifier) {
    val ctx = LocalContext.current
    val stats by RobotService.linkStats.collectAsState()
    val link by RobotService.linkFlow.collectAsState()
    val telemetry by (link?.telemetry
        ?: remember { kotlinx.coroutines.flow.MutableStateFlow(
            com.vacuum.nav.robot.Telemetry()) }).collectAsState()
    val mapSnap by (RobotService.mapper?.snapshot
        ?: remember { kotlinx.coroutines.flow.MutableStateFlow(null) })
        .collectAsState()
    val navState by (RobotService.nav?.state
        ?: remember { kotlinx.coroutines.flow.MutableStateFlow(
            com.vacuum.nav.robot.NavUiState()) }).collectAsState()
    var copied by remember { mutableStateOf(false) }

    // ---- firmware update by URL (WFO1, docs/PROTOCOL.md §3.10) ----------
    var otaUrl by remember { mutableStateOf("") }
    var otaPass by remember { mutableStateOf("") }
    var otaStatus by remember { mutableStateOf<String?>(null) }
    LaunchedEffect(link) {
        link?.adminReplies?.collect { d ->
            when (String(d, 0, minOf(4, d.size), Charsets.US_ASCII)) {
                "WFOA" -> {
                    val r = Packets.decodeWfoa(d) ?: return@collect
                    otaStatus = when (r.status) {
                        0 -> "Accepted - downloading while the robot is still"
                        1 -> "Denied: not connected as owner (reconnect)"
                        2 -> "Wrong OTA password"
                        3 -> "Stop the robot first"
                        4 -> "Bad URL (must start with http:// or https://)"
                        5 -> "OTA disabled on this firmware"
                        7 -> "An update is already running"
                        else -> "Rejected (code ${r.status})"
                    }
                }
                "WFOB" -> {
                    val r = Packets.decodeWfob(d) ?: return@collect
                    otaStatus = when (r.status) {
                        0 -> "Flashed - the robot reboots now; the link returns after ~10 s"
                        1 -> "Download failed (code ${r.detail}) - old firmware kept"
                        2 -> "No update available at that URL"
                        else -> "Failed (code ${r.status}/${r.detail})"
                    }
                }
            }
        }
    }

    val cfg = remember { RobotConfig() }
    val report = buildString {
        appendLine("VacuumNav diagnostics ${System.currentTimeMillis()}")
        appendLine("link: ${stats.transport}  scan ${stats.scanHz} Hz  " +
            "telem ${stats.telemHz} Hz")
        appendLine("ages: scan ${stats.lastScanAgeMs} ms  " +
            "telem ${stats.lastTelemAgeMs} ms")
        appendLine("battery: ${"%.2f".format(telemetry.batteryV)} V  " +
            "sonar F/L/R ${telemetry.sonarFrontM}/${telemetry.sonarLeftM}/" +
            "${telemetry.sonarRightM} m")
        appendLine("ticks L/R: ${telemetry.ticksL}/${telemetry.ticksR}  " +
            "rpm ${telemetry.rpmL}/${telemetry.rpmR}")
        appendLine("runs: ${RunStats.runs}  area ${RunStats.areaText()}  " +
            "time ${RunStats.timeText()}")
        appendLine("map: ${mapSnap?.let { "${it.width}x${it.height} v${it.version}" } ?: "none"}")
        appendLine("nav: ${com.vacuum.nav.robot.NavState.label(navState.state)}  " +
            "explore=${RobotService.exploreActive}  " +
            "dock=${RobotService.dockPhase}")
        appendLine("config: wheel ${cfg.wheelRadiusM} m  track " +
            "${cfg.trackWidthM} m  bins ${cfg.scanBins}  mirror " +
            "${cfg.lidarMirror} yaw ${cfg.lidarMountYawDeg}")
        appendLine("ports: scan ${cfg.boardScanPort} cmd " +
            "${cfg.boardCmdPort} telem ${cfg.boardTelemPort}")
    }

    Scaffold(modifier = modifier) { padding ->
        Column(
            Modifier.padding(padding).fillMaxSize()
                .verticalScroll(rememberScrollState()),
        ) {
            Row(
                Modifier.fillMaxWidth().padding(horizontal = 8.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                TopAppBar(
                title = { Text("Diagnostics", fontSize = Ts.Display, fontWeight = FontWeight.Bold) },
                navigationIcon = {
                    IconButton(onClick = onBack) {
                        Icon(Icons.AutoMirrored.Filled.ArrowBack, contentDescription = "Back")
                    }
                },
            )
            }

            Section("Link") {
                Text("Transport: ${stats.transport}", fontSize = Ts.Body)
                Text(
                    "Lidar: ${stats.scanHz} Hz · Telemetry: " +
                        "${stats.telemHz} Hz",
                    fontSize = Ts.Body,
                )
                val scanAge = stats.lastScanAgeMs
                Text(
                    "Last scan: " + when {
                        scanAge < 0 -> "never"
                        // Stale ages show in seconds (ms is unreadable
                        // past 3 s); both branches interpolate the value.
                        scanAge > 3000 ->
                            "%.1f s ago (stale)".format(scanAge / 1000.0)
                        else -> "$scanAge ms ago"
                    },
                    fontSize = Ts.Body,
                )
                Text(
                    "Last telemetry: " + when {
                        stats.lastTelemAgeMs < 0 -> "never"
                        stats.lastTelemAgeMs > 1000 ->
                            "${stats.lastTelemAgeMs} ms ago (stale)"
                        else -> "${stats.lastTelemAgeMs} ms ago"
                    },
                    fontSize = Ts.Body,
                )
            }

            Section("Robot") {
                Text(
                    "Battery: ${"%.2f".format(telemetry.batteryV)} V",
                    fontSize = Ts.Body,
                )
                Text(
                    "Sonar F ${fmtM(telemetry.sonarFrontM)} · " +
                        "L ${fmtM(telemetry.sonarLeftM)} · " +
                        "R ${fmtM(telemetry.sonarRightM)}",
                    fontSize = Ts.Body,
                )
                Text(
                    "Ticks ${telemetry.ticksL}/${telemetry.ticksR} · " +
                        "RPM ${telemetry.rpmL}/${telemetry.rpmR}",
                    fontSize = Ts.Body,
                )
                Text(
                    "Runs ${RunStats.runs} · ${RunStats.areaText()} · " +
                        RunStats.timeText(),
                    fontSize = Ts.Body,
                )
            }

            Section("Map & Nav") {
                Text(
                    "Map: " + (mapSnap?.let {
                        "${it.width}×${it.height} cells · keyframes " +
                            it.version
                    } ?: "not built yet"),
                    fontSize = Ts.Body,
                )
                Text(
                    "Nav: " +
                        com.vacuum.nav.robot.NavState.label(navState.state) +
                        " · explore=${RobotService.exploreActive} · " +
                        "dock=${RobotService.dockPhase}",
                    fontSize = Ts.Body,
                )
                Text(
                    "Queue: ${RobotService.coverageQueue.size} waypoints · " +
                        "rooms ${RobotService.selectedRooms}",
                    fontSize = Ts.Body,
                )
            }

            Section("Config (RobotConfig)") {
                Text(
                    "wheel r ${cfg.wheelRadiusM} m · track " +
                        "${cfg.trackWidthM} m · ticks ${cfg.ticksPerRev}",
                    fontSize = Ts.Body,
                )
                Text(
                    "lidar ${cfg.scanBins} bins · mirror ${cfg.lidarMirror} · " +
                        "yaw ${cfg.lidarMountYawDeg}° · " +
                        "range ${cfg.lidarMinRangeM} to  m",
                    fontSize = Ts.Body,
                )
                Text(
                    "limits ${cfg.maxLinearMps} m/s · " +
                        "${cfg.maxAngularRps} rad/s · ports " +
                        "${cfg.boardScanPort}/${cfg.boardCmdPort}/" +
                        "${cfg.boardTelemPort}",
                    fontSize = Ts.Body,
                )
            }

            Section("Firmware (OTA)") {
                Text(
                    "The robot pulls the image from a URL it can reach " +
                        "(docs/PROTOCOL.md §3.10). Password = OTA_PASSWORD.",
                    fontSize = Ts.Caption,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                Spacer(Modifier.height(Sp.Xs))
                OutlinedTextField(
                    value = otaUrl,
                    onValueChange = { otaUrl = it },
                    singleLine = true,
                    label = { Text("Firmware URL (.bin)") },
                    placeholder = { Text("https://…/vacuum_phone_esp32.ino.bin") },
                    modifier = Modifier.fillMaxWidth(),
                )
                Spacer(Modifier.height(Sp.Xs))
                OutlinedTextField(
                    value = otaPass,
                    onValueChange = { otaPass = it },
                    singleLine = true,
                    label = { Text("OTA password") },
                    visualTransformation = PasswordVisualTransformation(),
                    modifier = Modifier.fillMaxWidth(),
                )
                Spacer(Modifier.height(Sp.Xs))
                Button(
                    onClick = {
                        val l = link
                        val sent = l != null &&
                            l.send(Packets.buildWfo1(otaUrl.trim(), otaPass))
                        otaStatus = if (sent)
                            "Sent - waiting for the board's answer..."
                        else
                            "No robot link - connect via Wi-Fi or Bluetooth first"
                    },
                    enabled = otaUrl.startsWith("http") &&
                        otaStatus?.startsWith("Sent") != true,
                    modifier = Modifier.fillMaxWidth(),
                ) { Text("Update firmware") }
                otaStatus?.let {
                    Spacer(Modifier.height(Sp.Xs))
                    Text(it, fontSize = Ts.Caption,
                        color = MaterialTheme.colorScheme.primary)
                }
                Spacer(Modifier.height(Sp.Xs))
                Text(
                    "Robot must stand still; wheels stay locked during the " +
                        "download. The link and telemetry keep running - only " +
                        "lidar scans pause. On success the board reboots.",
                    fontSize = Ts.Caption,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }

            Row(
                Modifier.fillMaxWidth().padding(Sp.Md),
                horizontalArrangement =
                    androidx.compose.foundation.layout.Arrangement.End,
            ) {
                TextButton(onClick = {
                    val cm = ctx.getSystemService(
                        Context.CLIPBOARD_SERVICE) as ClipboardManager
                    cm.setPrimaryClip(ClipData.newPlainText("diagnostics",
                        report))
                    copied = true
                }) {
                    Text(if (copied) "Copied" else "Copy report")
                }
            }
            Spacer(Modifier.height(24.dp))
        }
    }
}

private fun fmtM(m: Float) = if (m <= 0f) "-" else "%.2f m".format(m)

@Composable
private fun Section(title: String, content: @Composable () -> Unit) {
    Card(
        Modifier.fillMaxWidth().padding(horizontal = Sp.Md, vertical = Sp.Xs),
        shape = RoundedCornerShape(Rd.M),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surfaceVariant),
    ) {
        Column(Modifier.padding(Sp.Md)) {
            Text(title, fontSize = Ts.Body, fontWeight = FontWeight.SemiBold,
                color = MaterialTheme.colorScheme.primary)
            Spacer(Modifier.height(Sp.Xs))
            content()
        }
    }
}
