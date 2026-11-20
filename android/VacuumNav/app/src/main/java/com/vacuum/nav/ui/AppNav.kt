package com.vacuum.nav.ui

import com.vacuum.nav.ui.theme.Sp
import com.vacuum.nav.ui.theme.Ts

import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Block
import androidx.compose.material.icons.filled.Gamepad
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Map
import androidx.compose.material.icons.filled.Schedule
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import com.vacuum.nav.robot.LinkState
import com.vacuum.nav.service.RobotService

/**
 * Bottom-navigation shell: Home / Map / Drive / Zones / Schedule - the
 * reference app's information architecture (welcome home -> enter map ->
 * routines, with zones and scheduling one tap away). Maps and History are
 * sub-screens opened from Home (a7th tab would not fit).
 *
 * Phase 3 of the redesign (design-baseline FINDING-005): an app-wide error
 * banner sits above the active screen whenever the link reports
 * ERROR_LINK - the state existed in code but no screen ever showed it.
 */
@Composable
fun AppNav() {
    var tab by rememberSaveable { mutableStateOf("home") }
    // "" | "maps" | "history" | "diagnostics"
    var sub by rememberSaveable { mutableStateOf("") }

    val linkState by (RobotService.link?.state
        ?: remember { kotlinx.coroutines.flow.MutableStateFlow(
            LinkState.CLAIMED) }).collectAsState()

    Scaffold(
        bottomBar = {
            NavigationBar {
                listOf(
                    "home" to "Home",
                    "map" to "Map",
                    "drive" to "Drive",
                    "zones" to "Zones",
                    "schedule" to "Schedule",
                ).forEach { (key, label) ->
                    NavigationBarItem(
                        selected = tab == key && sub.isEmpty(),
                        onClick = {
                            tab = key
                            sub = ""
                        },
                        icon = {
                            Icon(
                                imageVector = when (key) {
                                    "home" -> Icons.Filled.Home
                                    "map" -> Icons.Filled.Map
                                    "drive" -> Icons.Filled.Gamepad
                                    "zones" -> Icons.Filled.Block
                                    else -> Icons.Filled.Schedule
                                },
                                contentDescription = null,
                            )
                        },
                        label = { Text(label) },
                    )
                }
            }
        },
    ) { padding ->
        Column(Modifier.padding(padding).fillMaxSize()) {
            androidx.compose.animation.AnimatedVisibility(
                visible = linkState == LinkState.ERROR_LINK,
                enter = androidx.compose.animation.fadeIn() +
                    androidx.compose.animation.expandVertically(),
                exit = androidx.compose.animation.fadeOut() +
                    androidx.compose.animation.shrinkVertically(),
            ) { LinkErrorBanner() }
            val content = Modifier.fillMaxSize()
            when {
                sub == "maps" -> MapsScreen(
                    onBack = { sub = "" },
                    modifier = content,
                )
                sub == "history" -> HistoryScreen(
                    onBack = { sub = "" },
                    modifier = content,
                )
                sub == "diagnostics" -> DiagnosticsScreen(
                    onBack = { sub = "" },
                    modifier = content,
                )
                tab == "home" -> HomeScreen(
                    onEnter = { tab = "map" },
                    onOpenSchedules = { tab = "schedule" },
                    onOpenMaps = { sub = "maps" },
                    onOpenHistory = { sub = "history" },
                    onOpenDiagnostics = { sub = "diagnostics" },
                    modifier = content,
                )
                tab == "map" -> MapScreen(
                    onOpenZones = { tab = "zones" },
                    onOpenDrive = { tab = "drive" },
                    modifier = content,
                )
                tab == "drive" -> DriveScreen(content)
                tab == "zones" -> ZonesScreen(content)
                else -> SchedulesScreen(content)
            }
        }
    }
}

/** Persistent error strip while the robot link is down (FINDING-005). */
@Composable
private fun LinkErrorBanner() {
    Surface(
        Modifier.fillMaxWidth(),
        color = MaterialTheme.colorScheme.error,
    ) {
        Text(
            "⚠ Robot link lost — commands stopped. Check Wi-Fi/Bluetooth.",
            color = Color.White,
            fontSize = Ts.Body,
            fontWeight = FontWeight.Medium,
            modifier = Modifier.padding(horizontal = Sp.Md, vertical = Sp.Xs),
        )
    }
}
