package com.vacuum.nav.ui

import androidx.compose.foundation.layout.padding
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier

/**
 * Bottom-navigation shell: Home / Map / Drive / Zones / Schedule - the
 * reference app's information architecture (welcome home -> enter map ->
 * routines, with zones and scheduling one tap away).
 */
@Composable
fun AppNav() {
    var tab by rememberSaveable { mutableStateOf("home") }

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
                        selected = tab == key,
                        onClick = { tab = key },
                        icon = {
                            Text(
                                when (key) {
                                    "home" -> "⌂"
                                    "map" -> "🗺"
                                    "drive" -> "🕹"
                                    "zones" -> "🚫"
                                    else -> "⏰"
                                }
                            )
                        },
                        label = { Text(label) },
                    )
                }
            }
        },
    ) { padding ->
        val content = Modifier.padding(padding)
        when (tab) {
            "home" -> HomeScreen(
                onEnter = { tab = "map" },
                onOpenSchedules = { tab = "schedule" },
                modifier = content,
            )
            "map" -> MapScreen(
                onOpenZones = { tab = "zones" },
                onOpenDrive = { tab = "drive" },
                modifier = content,
            )
            "drive" -> DriveScreen(content)
            "zones" -> ZonesScreen(content)
            else -> SchedulesScreen(content)
        }
    }
}
