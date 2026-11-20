package com.vacuum.nav.ui

import com.vacuum.nav.ui.theme.Rd
import com.vacuum.nav.ui.theme.Sp
import com.vacuum.nav.ui.theme.Ts

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import com.vacuum.nav.robot.BleRobotLink
import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.robot.UdpRobotLink
import com.vacuum.nav.service.RobotService

enum class Transport { SIM, UDP, BLE }

fun transportOf(link: com.vacuum.nav.robot.RobotLink?): Transport = when (link) {
    is com.vacuum.nav.robot.SimRobotLink -> Transport.SIM
    is UdpRobotLink -> Transport.UDP
    is BleRobotLink -> Transport.BLE
    else -> Transport.SIM
}

/**
 * Transport picker: SIM (simulator) / Wi-Fi (UDP v2, prompt §3) / BLE
 * (FriendBot-style GATT link - no router needed, works within ~10 m).
 * Switching never restarts the brain: RobotService.switchLink keeps the
 * control thread and lidar layer running across the change.
 */
@Composable
fun ConnectDialog(onDismiss: () -> Unit) {
    val ctx = LocalContext.current
    var tab by remember { mutableStateOf(transportOf(RobotService.link)) }
    var ip by remember { mutableStateOf("192.168.1.42") }
    var bleDevices by remember {
        mutableStateOf(emptyList<android.bluetooth.le.ScanResult>())
    }
    var scanning by remember { mutableStateOf(false) }

    val blePermLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions(),
    ) { granted ->
        if (granted.values.all { it }) startBleScan {
            bleDevices = it; scanning = false
        } else scanning = false
    }

    fun requireBlePermsAndScan() {
        val needed = if (Build.VERSION.SDK_INT >= 31) {
            listOf(Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            listOf(Manifest.permission.ACCESS_FINE_LOCATION)
        }.filter {
            ContextCompat.checkSelfPermission(ctx, it) !=
                PackageManager.PERMISSION_GRANTED
        }
        scanning = true
        if (needed.isEmpty()) startBleScan { bleDevices = it; scanning = false }
        else blePermLauncher.launch(needed.toTypedArray())
    }

    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Connect to robot") },
        text = {
            Column {
                Row(horizontalArrangement = Arrangement.spacedBy(Sp.Xs)) {
                    OutlinedButton(onClick = { tab = Transport.SIM }) {
                        Text("SIM",
                            fontWeight = if (tab == Transport.SIM)
                                FontWeight.Bold else FontWeight.Normal)
                    }
                    OutlinedButton(onClick = { tab = Transport.UDP }) {
                        Text("Wi-Fi",
                            fontWeight = if (tab == Transport.UDP)
                                FontWeight.Bold else FontWeight.Normal)
                    }
                    OutlinedButton(onClick = {
                        tab = Transport.BLE
                        if (bleDevices.isEmpty() && !scanning)
                            requireBlePermsAndScan()
                    }) {
                        Text("BLE",
                            fontWeight = if (tab == Transport.BLE)
                                FontWeight.Bold else FontWeight.Normal)
                    }
                }
                Spacer(Modifier.height(12.dp))
                when (tab) {
                    Transport.SIM -> {
                        Text("Drive the built-in simulator - no hardware needed.")
                        Spacer(Modifier.height(8.dp))
                        TextButton(onClick = {
                            RobotService.connectSim(); onDismiss()
                        }) { Text("Use simulator", fontWeight = FontWeight.Bold) }
                    }
                    Transport.UDP -> {
                        Text("Wi-Fi UDP (protocol v2): board IP or hoverboard.local")
                        Spacer(Modifier.height(8.dp))
                        OutlinedTextField(
                            value = ip,
                            onValueChange = { ip = it },
                            singleLine = true,
                            label = { Text("Board address") },
                            modifier = Modifier.fillMaxWidth(),
                        )
                        Spacer(Modifier.height(8.dp))
                        TextButton(onClick = {
                            RobotService.connectUdp(ip.trim()); onDismiss()
                        }) { Text("Connect", fontWeight = FontWeight.Bold) }
                    }
                    Transport.BLE -> {
                        Text(
                            "Bluetooth LE (FriendBot GATT): instant, no Wi-Fi " +
                                "network needed.",
                            fontSize = Ts.Caption,
                        )
                        Spacer(Modifier.height(8.dp))
                        Row(horizontalArrangement = Arrangement.spacedBy(Sp.Xs)) {
                            TextButton(onClick = { requireBlePermsAndScan() }) {
                                Text(if (scanning) "Scanning…" else "Scan again")
                            }
                        }
                        if (bleDevices.isEmpty() && !scanning) {
                            Text("No boards found - is the robot powered and " +
                                "Bluetooth on?", fontSize = Ts.Caption)
                        }
                        LazyColumn(Modifier.height(160.dp)) {
                            items(bleDevices) { r ->
                                TextButton(onClick = {
                                    RobotService.connectBle(r.device.address)
                                    onDismiss()
                                }) {
                                    Text(
                                        (r.device.name ?: "Unknown") + "\n" +
                                            r.device.address,
                                        fontSize = Ts.Body,
                                    )
                                }
                            }
                        }
                    }
                }
            }
        },
        confirmButton = {
            TextButton(onClick = onDismiss) { Text("Close") }
        },
    )
}

private fun startBleScan(onDone: (List<android.bluetooth.le.ScanResult>) -> Unit) {
    // The scan helper lives with the link (same service UUID filter).
    val appCtx = com.vacuum.nav.VacuumApp.instance
    if (appCtx == null) { onDone(emptyList()); return }
    BleRobotLink.scan(appCtx, 5000L, onDone)
}
