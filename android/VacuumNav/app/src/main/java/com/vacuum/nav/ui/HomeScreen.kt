package com.vacuum.nav.ui

import com.vacuum.nav.ui.theme.*

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
import androidx.compose.material3.Card
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.CardDefaults
import androidx.compose.foundation.Image
import androidx.compose.material.icons.filled.Add
import androidx.compose.ui.res.painterResource
import com.vacuum.nav.R
import androidx.compose.material.icons.filled.BatteryChargingFull
import androidx.compose.material.icons.filled.History
import androidx.compose.material.icons.filled.Map
import androidx.compose.material.icons.filled.Schedule
import androidx.compose.material.icons.filled.CleaningServices
import androidx.compose.material.icons.filled.Restaurant
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
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
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.vacuum.nav.data.RunStats
import com.vacuum.nav.data.ScheduleStore
import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.service.RobotService
import com.vacuum.nav.ui.theme.RobotOrange

/**
 * Home, in the shape of the reference app's "Smart App" screen: greeting +
 * robot card (battery, state, big Enter) + Routines row. Everything else
 * lives behind the bottom navigation (AppNav).
 */
@Composable
fun HomeScreen(
    onEnter: () -> Unit,
    onOpenSchedules: () -> Unit,
    onOpenMaps: () -> Unit,
    onOpenHistory: () -> Unit,
    onOpenDiagnostics: () -> Unit,
    modifier: Modifier = Modifier,
) {
    val link = RobotService.link
    val telemetry by (link?.telemetry
        ?: remember { kotlinx.coroutines.flow.MutableStateFlow(
            com.vacuum.nav.robot.Telemetry()) }).collectAsState()
    val autoClean = RobotService.autoClean
    val eStop = RobotService.eStop
    var showAbout by remember { mutableStateOf(false) }
    var showConnect by remember { mutableStateOf(false) }
    // Bumped after a transport switch so the chip re-reads RobotService.link.
    var linkVersion by remember { mutableStateOf(0) }
    val linkType = remember(linkVersion) { transportOf(RobotService.link) }

    val ctx = androidx.compose.ui.platform.LocalContext.current
    val nextScheduleText = remember {
        ScheduleStore.load(ctx).filter { it.enabled }
            .minByOrNull { it.hour * 60 + it.minute }?.timeText ?: "—"
    }

    val batteryPct = when {
        eStop -> "—"
        else -> {
            val v = telemetry.batteryV.toDouble()
            val cfg = RobotConfig()
            ((v - cfg.batteryCriticalV) /
                (cfg.batteryFullV - cfg.batteryCriticalV) * 100)
                .coerceIn(0.0, 100.0).toInt().toString() + "%"
        }
    }

    // Phase 3 of the redesign (FINDING-005): real loading state while the
    // foreground service spins up the brain.
    if (link == null) {
        Box(Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
            CircularProgressIndicator()
            Spacer(Modifier.height(Sp.Md))
            Text("Starting robot brain…", fontSize = Ts.Body)
        }
        return
    }

    Column(
        modifier
            .fillMaxSize()
            .background(
                Brush.verticalGradient(
                    listOf(
                        MaterialTheme.colorScheme.primary.copy(alpha = 0.08f),
                        MaterialTheme.colorScheme.background,
                    )
                )
            )
            .padding(20.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text("Welcome home!", fontSize = Ts.Display, fontWeight = FontWeight.Bold)
            Row(verticalAlignment = Alignment.CenterVertically) {
                // Transport chip: SIM / Wi-Fi / BLE (FriendBot-style link).
                TextButton(onClick = {
                    showConnect = true
                }) {
                    Text(
                        when (linkType) {
                            Transport.SIM -> "📱 SIM"
                            Transport.UDP -> "📶 Wi-Fi"
                            Transport.BLE -> "蓝牙 BLE"
                        },
                        fontSize = Ts.Body,
                        fontWeight = FontWeight.SemiBold,
                    )
                }
                TextButton(onClick = onOpenDiagnostics) {
                    Icon(Icons.Filled.Settings, contentDescription = "Diagnostics")
                }
            }
        }
        Spacer(Modifier.height(4.dp))
        Text(
            if (eStop) "Robot stopped — E-STOP pressed"
            else if (autoClean) "Cleaning now…"
            else "Ready to clean",
            fontSize = Ts.Body,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
        Spacer(Modifier.height(20.dp))

        // Robot card (reference: product shot + battery + Enter).
        Card(
            Modifier.fillMaxWidth(),
            shape = RoundedCornerShape(Rd.L),
            colors = CardDefaults.cardColors(
                containerColor = MaterialTheme.colorScheme.primaryContainer
            ),
            elevation = CardDefaults.cardElevation(6.dp),
        ) {
            Column(
                Modifier.fillMaxWidth().padding(Sp.Lg),
                horizontalAlignment = Alignment.CenterHorizontally,
            ) {
                Row(
                    Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                ) {
                    Text(
                        RobotConfig().robotName,
                        fontSize = Ts.Headline, fontWeight = FontWeight.SemiBold,
                        color = MaterialTheme.colorScheme.onPrimaryContainer,
                    )
                    Surface(
                        shape = RoundedCornerShape(50),
                        color = when {
                            eStop -> MaterialTheme.colorScheme.error
                            autoClean -> RobotOrange
                            else -> Positive
                        }.copy(alpha = 0.15f),
                    ) {
                        Text(
                            when {
                                eStop -> "E-STOP"
                                autoClean -> "CLEANING"
                                else -> "SIM · IDLE"
                            },
                            fontSize = Ts.Caption,
                            fontWeight = FontWeight.Bold,
                            color = when {
                                eStop -> MaterialTheme.colorScheme.error
                                autoClean -> RobotOrange
                                else -> Positive
                            },
                            modifier = Modifier.padding(
                                horizontal = 10.dp, vertical = 4.dp),
                        )
                    }
                }
                Spacer(Modifier.height(20.dp))
                // Robot "product shot": the app logo (same art as the
                // launcher icon) on a soft disc.
                Box(
                    Modifier.size(150.dp)
                        .background(
                            Brush.radialGradient(
                                listOf(HeroTop, HeroBottom)
                            ),
                            CircleShape,
                        ),
                    contentAlignment = Alignment.Center,
                ) {
                    Image(
                        painter = painterResource(
                            R.drawable.ic_launcher_foreground),
                        contentDescription = "VacuumNav logo",
                        modifier = Modifier.size(110.dp),
                    )
                }
                Spacer(Modifier.height(12.dp))
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Icon(
                        Icons.Filled.BatteryChargingFull,
                        contentDescription = null,
                        tint = if (eStop) MaterialTheme.colorScheme.error else MaterialTheme.colorScheme.onPrimaryContainer,
                        modifier = Modifier.size(28.dp),
                    )
                    Spacer(Modifier.width(8.dp))
                    Text(
                        batteryPct,
                        fontSize = Ts.XL, fontWeight = FontWeight.Bold,
                        color = if (eStop) MaterialTheme.colorScheme.error else MaterialTheme.colorScheme.onPrimaryContainer,
                    )
                }
                Text(
                    "Battery · ${"%.1f".format(telemetry.batteryV)} V",
                    fontSize = Ts.Body,
                    color = MaterialTheme.colorScheme.onPrimaryContainer.copy(alpha = 0.7f),
                )
                Spacer(Modifier.height(16.dp))
                Row(
                    Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(10.dp),
                ) {
                    StatTile("Last clean",
                        "${RunStats.areaText()} · ${RunStats.timeText()}",
                        Modifier.weight(1f))
                    StatTile("Next schedule", nextScheduleText, Modifier.weight(1f))
                }
                Spacer(Modifier.height(20.dp))
                Button(
                    onClick = onEnter,
                    Modifier.fillMaxWidth().height(52.dp),
                    shape = RoundedCornerShape(50),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = RobotOrange,
                    ),
                ) {
                    Text("Open 3D map", fontSize = Ts.Title,
                        fontWeight = FontWeight.SemiBold)
                }
            }
        }

        Spacer(Modifier.height(24.dp))
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
        ) {
            Text("Routines", fontSize = Ts.Title,
                fontWeight = FontWeight.SemiBold)
            TextButton(onClick = onOpenSchedules) { Text("See all") }
        }
        Spacer(Modifier.height(8.dp))
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(Sp.Sm),
        ) {
            RoutineChip(
                "After Meals",
                "Quick clean",
                Icons.Filled.Restaurant,
                Modifier.weight(1f),
            ) {
                RobotService.startCleaning()
                onEnter()
            }
            RoutineChip(
                "Full Cleaning",
                "Whole home",
                Icons.Filled.CleaningServices,
                Modifier.weight(1f),
            ) {
                RobotService.startCleaning()
                onEnter()
            }
            RoutineChip(
                "Add",
                "New routine",
                Icons.Filled.Add,
                Modifier.weight(1f),
                onClick = onOpenSchedules,
            )
        }

        Spacer(Modifier.height(16.dp))
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(Sp.Sm),
        ) {
            RoutineChip(
                "Maps",
                "Floors: save / load",
                Icons.Filled.Map,
                Modifier.weight(1f),
                onClick = onOpenMaps,
            )
            RoutineChip(
                "History",
                "Past runs",
                Icons.Filled.History,
                Modifier.weight(1f),
                onClick = onOpenHistory,
            )
            RoutineChip(
                "Schedules",
                "Timed cleans",
                Icons.Filled.Schedule,
                Modifier.weight(1f),
                onClick = onOpenSchedules,
            )
        }

        Spacer(Modifier.weight(1f))
        Text(
            "Stats · ${RunStats.areaText()} cleaned · " +
                "${RunStats.runs} runs · ${RunStats.timeText()}",
            fontSize = Ts.Caption,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }

    if (showConnect) {
        ConnectDialog(onDismiss = {
            showConnect = false
            linkVersion++   // refresh the chip after a possible switch
        })
    }
}

@Composable
private fun RoutineChip(
    title: String,
    subtitle: String,
    icon: ImageVector,
    modifier: Modifier = Modifier,
    onClick: () -> Unit,
) {
    Card(
        modifier = modifier,
        shape = RoundedCornerShape(Rd.L),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surfaceVariant
        ),
        onClick = onClick,
    ) {
        Column(Modifier.padding(Sp.Sm)) {
            Icon(icon, contentDescription = null, tint = RobotOrange,
                modifier = Modifier.size(20.dp))
            Spacer(Modifier.height(4.dp))
            Text(title, fontSize = Ts.Body, fontWeight = FontWeight.SemiBold)
            Spacer(Modifier.height(4.dp))
            Text(
                subtitle, fontSize = Ts.Caption,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
    }
}

@Composable
private fun AboutDialog(onDismiss: () -> Unit) {
    androidx.compose.material3.AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Settings") },
        text = {
            val c = RobotConfig()
            Column {
                Text("Robot: ${c.robotName} (SIM link)")
                Text("Wheel r = ${c.wheelRadiusM} m · track = ${c.trackWidthM} m")
                Text("Lidar: ${c.scanBins} bins, mirror=${c.lidarMirror}, " +
                    "yaw=${c.lidarMountYawDeg}°")
                Text("Limits: ${c.maxLinearMps} m/s, ${c.maxAngularRps} rad/s")
                Text("Ports: scan ${c.boardScanPort} · cmd ${c.boardCmdPort} " +
                    "· telem ${c.boardTelemPort}")
                Spacer(Modifier.height(8.dp))
                Text(
                    "Full RobotConfig editing + calibration wizard arrive " +
                        "with Phase 2 hardware.",
                    fontSize = Ts.Caption,
                )
            }
        },
        confirmButton = {
            TextButton(onClick = onDismiss) { Text("Close") }
        },
    )
}

@Composable
private fun StatTile(label: String, value: String, modifier: Modifier = Modifier) {
    Surface(
        modifier = modifier,
        shape = RoundedCornerShape(Rd.M),
        color = MaterialTheme.colorScheme.surface,
    ) {
        Column(Modifier.padding(Sp.Sm)) {
            Text(label, fontSize = Ts.Caption,
                color = MaterialTheme.colorScheme.onSurfaceVariant)
            Text(value, fontSize = Ts.Body, fontWeight = FontWeight.Medium)
        }
    }
}
