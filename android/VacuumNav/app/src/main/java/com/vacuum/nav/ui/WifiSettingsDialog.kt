package com.vacuum.nav.ui

import com.vacuum.nav.ui.theme.Rd
import com.vacuum.nav.ui.theme.Sp
import com.vacuum.nav.ui.theme.Ts

import android.Manifest
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.net.wifi.WifiManager
import android.os.Build
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Lock
import androidx.compose.material.icons.filled.Refresh
import androidx.compose.material.icons.filled.Wifi
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.unit.dp
import com.vacuum.nav.robot.protocol.Packets
import com.vacuum.nav.service.RobotService

/** One network from the phone's scan (deduped by SSID, strongest wins). */
private data class WifiNet(val ssid: String, val level: Int, val secured: Boolean)

private fun WifiManager.scanNow(): List<WifiNet> {
    // Cached results are fine to show immediately; Scan refreshes them.
    val bySsid = LinkedHashMap<String, WifiNet>()
    @Suppress("DEPRECATION")
    scanResults
        .filter { it.SSID.isNotEmpty() }              // drop hidden networks
        .sortedByDescending { it.level }
        .forEach { r ->
            val caps = r.capabilities ?: ""
            bySsid.putIfAbsent(
                r.SSID,
                WifiNet(
                    ssid = r.SSID,
                    level = r.level,
                    secured = caps.contains("WPA") || caps.contains("WEP") ||
                        caps.contains("PSK"),
                ),
            )
        }
    return bySsid.values.sortedByDescending { it.level }
}

/**
 * Robot Wi-Fi provisioning (docs/PROTOCOL.md §3.9): scan networks with the
 * PHONE's radio, pick one, send WFC1 over the live link (UDP or BLE - BLE
 * is the point: configure the network before the board has joined one).
 * The board saves to NVS and reconnects; WFCA is the ack.
 */
@Composable
fun WifiSettingsDialog(onDismiss: () -> Unit) {
    val ctx = androidx.compose.ui.platform.LocalContext.current
    val appCtx = ctx.applicationContext
    val wm = remember {
        appCtx.getSystemService(Context.WIFI_SERVICE) as WifiManager
    }
    val link = RobotService.link
    val canSend = link != null && transportOf(link) != Transport.SIM

    var networks by remember { mutableStateOf(emptyList<WifiNet>()) }
    var selected by remember { mutableStateOf<WifiNet?>(null) }
    var pass by remember { mutableStateOf("") }
    var status by remember { mutableStateOf<String?>(null) }
    var scanning by remember { mutableStateOf(false) }

    // Permission: NEARBY_WIFI_DEVICES on 33+, fine location before that
    // (WiFi scan results are location-gated below 33).
    val permNeeded = if (Build.VERSION.SDK_INT >= 33)
        Manifest.permission.NEARBY_WIFI_DEVICES
    else
        Manifest.permission.ACCESS_FINE_LOCATION
    val permLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.RequestPermission(),
    ) { granted ->
        if (granted) {
            scanning = true
            wm.startScan()
        } else {
            status = "Permission needed to list nearby networks"
        }
    }
    fun havePerm(): Boolean =
        ContextCompat_check(appCtx, permNeeded)

    fun refresh() {
        if (!havePerm()) {
            permLauncher.launch(permNeeded)
            return
        }
        networks = wm.scanNow()
        scanning = true
        @Suppress("DEPRECATION")
        wm.startScan()   // results arrive via the receiver below
    }

    // Scan results broadcast -> update the list.
    DisposableEffect(Unit) {
        val receiver = object : BroadcastReceiver() {
            override fun onReceive(c: Context, intent: Intent) {
                networks = wm.scanNow()
                scanning = false
            }
        }
        appCtx.registerReceiver(
            receiver, IntentFilter(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION),
        )
        onDispose { runCatching { appCtx.unregisterReceiver(receiver) } }
    }

    // First open: cached results if permitted, else ask.
    LaunchedEffect(Unit) {
        if (havePerm()) networks = wm.scanNow() else permLauncher.launch(permNeeded)
    }

    // WFCA ack from the board.
    LaunchedEffect(link) {
        link?.adminReplies?.collect { d ->
            if (d.size >= 5 && String(d, 0, 4, Charsets.US_ASCII) == "WFCA") {
                val r = Packets.decodeWfca(d) ?: return@collect
                status = when (r.status) {
                    0 -> "Saved - the robot is reconnecting now"
                    1 -> "Rejected: bad request"
                    2 -> "Rejected: not connected as owner (reconnect)"
                    4 -> "This firmware build is AP-only (no STA join)"
                    else -> "Rejected (code ${r.status})"
                }
            }
        }
    }

    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Robot Wi-Fi network") },
        text = {
            Column {
                Text(
                    "Join any router network. Saved on the robot (NVS), " +
                        "survives reboots and firmware updates.",
                    fontSize = Ts.Caption,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
                Spacer(Modifier.height(Sp.Sm))

                Row(verticalAlignment = Alignment.CenterVertically) {
                    Text("Networks nearby", fontSize = Ts.Body,
                        fontWeight = androidx.compose.ui.text.font.FontWeight.Medium)
                    Spacer(Modifier.weight(1f))
                    IconButton(onClick = { refresh() }, enabled = !scanning) {
                        Icon(Icons.Filled.Refresh, contentDescription = "Rescan")
                    }
                }

                if (networks.isEmpty()) {
                    Text(
                        if (scanning) "Scanning..."
                        else "No networks found - tap refresh",
                        fontSize = Ts.Caption,
                        color = MaterialTheme.colorScheme.onSurfaceVariant,
                        modifier = Modifier.padding(vertical = Sp.Xs),
                    )
                } else {
                    Column(
                        Modifier
                            .heightIn(max = 180.dp)
                            .verticalScroll(rememberScrollState()),
                    ) {
                        networks.forEach { net ->
                            val isSel = selected?.ssid == net.ssid
                            TextButton(
                                onClick = {
                                    selected = net
                                    pass = ""
                                    status = null
                                },
                                modifier = Modifier.fillMaxWidth(),
                            ) {
                                Icon(
                                    Icons.Filled.Wifi,
                                    contentDescription = null,
                                    modifier = Modifier
                                        .padding(end = Sp.Xs)
                                        .height(16.dp),
                                )
                                Text(
                                    net.ssid,
                                    fontSize = Ts.Body,
                                    fontWeight = if (isSel)
                                        androidx.compose.ui.text.font.FontWeight.Bold
                                    else
                                        androidx.compose.ui.text.font.FontWeight.Normal,
                                    modifier = Modifier.weight(1f),
                                )
                                Text(
                                    "${net.level} dBm",
                                    fontSize = Ts.Caption,
                                    color = MaterialTheme.colorScheme
                                        .onSurfaceVariant,
                                )
                                if (net.secured) {
                                    Icon(
                                        Icons.Filled.Lock,
                                        contentDescription = "secured",
                                        modifier = Modifier
                                            .padding(start = Sp.Xs)
                                            .height(14.dp),
                                    )
                                }
                            }
                        }
                    }
                }

                selected?.let { net ->
                    Spacer(Modifier.height(Sp.Xs))
                    if (net.secured) {
                        OutlinedTextField(
                            value = pass,
                            onValueChange = { pass = it },
                            singleLine = true,
                            label = { Text("Password") },
                            visualTransformation = PasswordVisualTransformation(),
                            modifier = Modifier.fillMaxWidth(),
                        )
                        Spacer(Modifier.height(Sp.Xs))
                    }
                    Button(
                        onClick = {
                            if (!canSend) {
                                status = "Connect to the robot first " +
                                    "(Wi-Fi or Bluetooth tab)"
                                return@Button
                            }
                            status = "Sending..."
                            val ok = link!!.send(
                                Packets.buildWfc1(net.ssid, pass),
                            )
                            if (!ok) status = "No robot link to send on"
                        },
                        enabled = status?.startsWith("Sending") != true,
                        modifier = Modifier.fillMaxWidth(),
                    ) {
                        Text("Save on robot")
                    }
                }

                status?.let {
                    Spacer(Modifier.height(Sp.Xs))
                    Text(
                        it,
                        fontSize = Ts.Caption,
                        color = MaterialTheme.colorScheme.primary,
                    )
                }

                Spacer(Modifier.height(Sp.Xs))
                Text(
                    "Bluetooth keeps working while the robot switches. " +
                        "If you talk over the robot's own Wi-Fi, its AP " +
                        "stays open in fallback mode, so the link survives too.",
                    fontSize = Ts.Caption,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
        },
        confirmButton = {
            TextButton(onClick = onDismiss) { Text("Close") }
        },
        dismissButton = {
            OutlinedButton(
                onClick = { selected = null; pass = ""; status = null },
            ) { Text("Clear") }
        },
    )
}

// ContextCompat.checkSelfPermission without pulling the whole androidx.core
// import set into this file's read path twice.
private fun ContextCompat_check(ctx: Context, perm: String): Boolean =
    ctx.checkSelfPermission(perm) >= android.content.pm.PackageManager
        .PERMISSION_GRANTED
