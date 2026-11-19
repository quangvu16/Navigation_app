package com.vacuum.nav.ui

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
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
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
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.vacuum.nav.data.RunStats
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
            Text("Welcome home!", fontSize = 24.sp, fontWeight = FontWeight.Bold)
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
                        fontSize = 13.sp,
                        fontWeight = FontWeight.SemiBold,
                    )
                }
                TextButton(onClick = { showAbout = true }) { Text("⚙") }
            }
        }
        Spacer(Modifier.height(4.dp))
        Text(
            if (eStop) "Robot stopped — E-STOP pressed"
            else if (autoClean) "Cleaning now…"
            else "Ready to clean",
            fontSize = 14.sp,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
        Spacer(Modifier.height(20.dp))

        // Robot card (reference: product shot + battery + Enter).
        Card(
            Modifier.fillMaxWidth(),
            shape = RoundedCornerShape(24.dp),
            colors = CardDefaults.cardColors(
                containerColor = Color(0xFF14213D)
            ),
            elevation = CardDefaults.cardElevation(6.dp),
        ) {
            Column(
                Modifier.fillMaxWidth().padding(24.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
            ) {
                Row(
                    Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                ) {
                    Text(
                        RobotConfig().robotName,
                        fontSize = 18.sp, fontWeight = FontWeight.SemiBold,
                        color = Color.White,
                    )
                    Surface(
                        shape = RoundedCornerShape(50),
                        color = when {
                            eStop -> MaterialTheme.colorScheme.error
                            autoClean -> RobotOrange
                            else -> Color(0xFF7BE0B0)
                        }.copy(alpha = 0.15f),
                    ) {
                        Text(
                            when {
                                eStop -> "E-STOP"
                                autoClean -> "CLEANING"
                                else -> "SIM · IDLE"
                            },
                            fontSize = 12.sp,
                            fontWeight = FontWeight.Bold,
                            color = when {
                                eStop -> MaterialTheme.colorScheme.error
                                autoClean -> RobotOrange
                                else -> Color(0xFF7BE0B0)
                            },
                            modifier = Modifier.padding(
                                horizontal = 10.dp, vertical = 4.dp),
                        )
                    }
                }
                Spacer(Modifier.height(20.dp))
                // Robot "product shot": a disc with the heading arrow.
                Box(
                    Modifier.size(150.dp)
                        .background(
                            Brush.radialGradient(
                                listOf(Color(0xFF37474F), Color(0xFF263238))
                            ),
                            CircleShape,
                        ),
                    contentAlignment = Alignment.Center,
                ) {
                    Text("🛞", fontSize = 64.sp)
                }
                Spacer(Modifier.height(12.dp))
                Text(
                    "🔋 $batteryPct",
                    fontSize = 28.sp, fontWeight = FontWeight.Bold,
                    color = if (eStop) MaterialTheme.colorScheme.error
                    else Color.White,
                )
                Text(
                    "Battery · ${"%.1f".format(telemetry.batteryV)} V",
                    fontSize = 13.sp,
                    color = Color.White.copy(alpha = 0.7f),
                )
                Spacer(Modifier.height(20.dp))
                Button(
                    onClick = onEnter,
                    Modifier.fillMaxWidth().height(52.dp),
                    shape = RoundedCornerShape(50),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = Color(0xFF2BB3A0),
                        contentColor = Color(0xFF06201C),
                    ),
                ) {
                    Text("Enter", fontSize = 17.sp,
                        fontWeight = FontWeight.SemiBold)
                }
            }
        }

        Spacer(Modifier.height(24.dp))
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
        ) {
            Text("Routines", fontSize = 16.sp,
                fontWeight = FontWeight.SemiBold)
            TextButton(onClick = onOpenSchedules) { Text("See all") }
        }
        Spacer(Modifier.height(8.dp))
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            RoutineChip(
                "🍽 After Meals",
                "Quick clean",
                Modifier.weight(1f),
            ) {
                RobotService.startCleaning()
                onEnter()
            }
            RoutineChip(
                "🏠 Full Cleaning",
                "Whole home",
                Modifier.weight(1f),
            ) {
                RobotService.startCleaning()
                onEnter()
            }
            RoutineChip(
                "＋ Add",
                "New routine",
                Modifier.weight(1f),
                onClick = onOpenSchedules,
            )
        }

        Spacer(Modifier.weight(1f))
        Text(
            "Stats · ${RunStats.areaText()} cleaned · " +
                "${RunStats.runs} runs · ${RunStats.timeText()}",
            fontSize = 12.sp,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )
    }

    if (showAbout) {
        AboutDialog(onDismiss = { showAbout = false })
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
    modifier: Modifier = Modifier,
    onClick: () -> Unit,
) {
    Card(
        modifier = modifier,
        shape = RoundedCornerShape(20.dp),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surfaceVariant
        ),
        onClick = onClick,
    ) {
        Column(Modifier.padding(12.dp)) {
            Text(title, fontSize = 13.sp, fontWeight = FontWeight.SemiBold)
            Spacer(Modifier.height(4.dp))
            Text(
                subtitle, fontSize = 11.sp,
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
                    fontSize = 12.sp,
                )
            }
        },
        confirmButton = {
            TextButton(onClick = onDismiss) { Text("Close") }
        },
    )
}
