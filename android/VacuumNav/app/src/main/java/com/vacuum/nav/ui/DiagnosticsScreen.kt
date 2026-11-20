package com.vacuum.nav.ui

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
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.vacuum.nav.data.RunStats
import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.service.RobotService

/**
 * Diagnostics (prompt §6): packet rates, ages, battery, sonar, map/nav
 * state, robot config, and a copyable text report (the "log export" for
 * now; file export lands with the share-sheet polish pass). Hardware
 * items (mirror test, wheel calibration wizard) light up when the real
 * link is connected.
 */
@Composable
fun DiagnosticsScreen(onBack: () -> Unit, modifier: Modifier = Modifier) {
    val ctx = LocalContext.current
    val stats by RobotService.linkStats.collectAsState()
    val telemetry by (RobotService.link?.telemetry
        ?: remember { kotlinx.coroutines.flow.MutableStateFlow(
            com.vacuum.nav.robot.Telemetry()) }).collectAsState()
    val mapSnap by (RobotService.mapper?.snapshot
        ?: remember { kotlinx.coroutines.flow.MutableStateFlow(null) })
        .collectAsState()
    val navState by (RobotService.nav?.state
        ?: remember { kotlinx.coroutines.flow.MutableStateFlow(
            com.vacuum.nav.robot.NavUiState()) }).collectAsState()
    var copied by remember { mutableStateOf(false) }

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
                TextButton(onClick = onBack) { Text("‹ Back") }
                Text("Diagnostics", fontSize = 22.sp,
                    fontWeight = FontWeight.Bold)
            }

            Section("Link") {
                Text("Transport: ${stats.transport}", fontSize = 14.sp)
                Text(
                    "Lidar: ${stats.scanHz} Hz · Telemetry: " +
                        "${stats.telemHz} Hz",
                    fontSize = 14.sp,
                )
                val scanAge = stats.lastScanAgeMs
                Text(
                    "Last scan: " + when {
                        scanAge < 0 -> "never"
                        scanAge > 3000 -> "${scanAge / 1000} s ago ⚠"
                        else -> "$scanAge ms ago"
                    },
                    fontSize = 14.sp,
                )
                Text(
                    "Last telemetry: " + when {
                        stats.lastTelemAgeMs < 0 -> "never"
                        stats.lastTelemAgeMs > 1000 ->
                            "${stats.lastTelemAgeMs} ms ago ⚠"
                        else -> "${stats.lastTelemAgeMs} ms ago"
                    },
                    fontSize = 14.sp,
                )
            }

            Section("Robot") {
                Text(
                    "Battery: ${"%.2f".format(telemetry.batteryV)} V",
                    fontSize = 14.sp,
                )
                Text(
                    "Sonar F ${fmtM(telemetry.sonarFrontM)} · " +
                        "L ${fmtM(telemetry.sonarLeftM)} · " +
                        "R ${fmtM(telemetry.sonarRightM)}",
                    fontSize = 14.sp,
                )
                Text(
                    "Ticks ${telemetry.ticksL}/${telemetry.ticksR} · " +
                        "RPM ${telemetry.rpmL}/${telemetry.rpmR}",
                    fontSize = 14.sp,
                )
                Text(
                    "Runs ${RunStats.runs} · ${RunStats.areaText()} · " +
                        RunStats.timeText(),
                    fontSize = 14.sp,
                )
            }

            Section("Map & Nav") {
                Text(
                    "Map: " + (mapSnap?.let {
                        "${it.width}×${it.height} cells · keyframes " +
                            it.version
                    } ?: "not built yet"),
                    fontSize = 14.sp,
                )
                Text(
                    "Nav: " +
                        com.vacuum.nav.robot.NavState.label(navState.state) +
                        " · explore=${RobotService.exploreActive} · " +
                        "dock=${RobotService.dockPhase}",
                    fontSize = 14.sp,
                )
                Text(
                    "Queue: ${RobotService.coverageQueue.size} waypoints · " +
                        "rooms ${RobotService.selectedRooms}",
                    fontSize = 14.sp,
                )
            }

            Section("Config (RobotConfig)") {
                Text(
                    "wheel r ${cfg.wheelRadiusM} m · track " +
                        "${cfg.trackWidthM} m · ticks ${cfg.ticksPerRev}",
                    fontSize = 13.sp,
                )
                Text(
                    "lidar ${cfg.scanBins} bins · mirror ${cfg.lidarMirror} · " +
                        "yaw ${cfg.lidarMountYawDeg}° · " +
                        "range ${cfg.lidarMinRangeM}–${cfg.lidarMaxRangeM} m",
                    fontSize = 13.sp,
                )
                Text(
                    "limits ${cfg.maxLinearMps} m/s · " +
                        "${cfg.maxAngularRps} rad/s · ports " +
                        "${cfg.boardScanPort}/${cfg.boardCmdPort}/" +
                        "${cfg.boardTelemPort}",
                    fontSize = 13.sp,
                )
            }

            Row(
                Modifier.fillMaxWidth().padding(16.dp),
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
                    Text(if (copied) "Copied ✓" else "Copy report")
                }
            }
            Spacer(Modifier.height(24.dp))
        }
    }
}

private fun fmtM(m: Float) = if (m <= 0f) "—" else "%.2f m".format(m)

@Composable
private fun Section(title: String, content: @Composable () -> Unit) {
    Card(
        Modifier.fillMaxWidth().padding(horizontal = 16.dp, vertical = 6.dp),
        shape = RoundedCornerShape(16.dp),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surfaceVariant),
    ) {
        Column(Modifier.padding(16.dp)) {
            Text(title, fontSize = 13.sp, fontWeight = FontWeight.SemiBold,
                color = MaterialTheme.colorScheme.primary)
            Spacer(Modifier.height(6.dp))
            content()
        }
    }
}
