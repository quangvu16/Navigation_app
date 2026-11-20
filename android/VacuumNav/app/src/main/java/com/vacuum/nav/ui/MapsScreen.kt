package com.vacuum.nav.ui

import com.vacuum.nav.ui.theme.Rd
import com.vacuum.nav.ui.theme.Sp
import com.vacuum.nav.ui.theme.Ts

import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.FloatingActionButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.Text
import androidx.compose.material3.TopAppBar
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.ArrowBack
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.getValue
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.vacuum.nav.data.MapMeta
import com.vacuum.nav.data.MapExport
import com.vacuum.nav.data.MapStore
import com.vacuum.nav.service.RobotService
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Phase 8: multi-floor maps (prompt §6 screens: save, load, switch,
 * rename, delete). "Save current" snapshots the live mapper output as a
 * new floor; "Load" installs it as the base layer AND into the nav
 * costmap, so go-to / rooms / exploration work on that floor immediately.
 */
// TopAppBar is still marked experimental in this material3 build.
@OptIn(androidx.compose.material3.ExperimentalMaterial3Api::class)
@Composable
fun MapsScreen(onBack: () -> Unit, modifier: Modifier = Modifier) {
    val ctx = LocalContext.current
    val maps = remember {
        mutableStateListOf<MapMeta>().apply { addAll(MapStore.list(ctx)) }
    }
    var editing by remember { mutableStateOf<MapMeta?>(null) }
    var newName by remember { mutableStateOf("") }
    var status by remember { mutableStateOf<String?>(null) }
    val fmt = remember { SimpleDateFormat("yyyy-MM-dd HH:mm", Locale.US) }

    // SAF picker for map JSON import (no storage permission needed).
    val importLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.OpenDocument(),
    ) { uri ->
        if (uri != null) {
            val meta = MapExport.import(ctx, uri)
            if (meta != null) {
                maps.add(meta)
                status = "Imported ${meta.name}"
            } else {
                status = "Import failed: not a VacuumNav map file"
            }
        }
    }

    Scaffold(
        modifier = modifier,
        floatingActionButton = {
            FloatingActionButton(
                onClick = {
                    val snap = RobotService.mapper?.snapshot?.value
                    if (snap == null) {
                        status = "Nothing mapped yet. Explore first."
                    } else {
                        val floor = (maps.maxOfOrNull { it.floor } ?: 0) + 1
                        val meta = MapStore.save(ctx, "Floor $floor", floor,
                            snap)
                        maps.add(meta)
                        status = "Saved ${meta.name}"
                    }
                },
                containerColor = MaterialTheme.colorScheme.primary,
            ) { Icon(androidx.compose.material.icons.Icons.Filled.Save, contentDescription = "Save map", tint = MaterialTheme.colorScheme.onPrimary) }
        },
    ) { padding ->
        Column(Modifier.padding(padding).fillMaxSize()) {
            Row(
                Modifier.fillMaxWidth().padding(horizontal = 8.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                TopAppBar(
                title = { Text("Maps", fontSize = Ts.Display, fontWeight = FontWeight.Bold) },
                navigationIcon = {
                    IconButton(onClick = onBack) {
                        Icon(Icons.AutoMirrored.Filled.ArrowBack, contentDescription = "Back")
                    }
                },
                actions = {
                    TextButton(onClick = {
                        importLauncher.launch(arrayOf("application/json", "*/*"))
                    }) { Text("Import", fontSize = Ts.Body) }
                },
            )
            }
            status?.let {
                Text(
                    it, fontSize = Ts.Body,
                    color = MaterialTheme.colorScheme.primary,
                    modifier = Modifier.padding(horizontal = Sp.Md),
                )
            }
            if (maps.isEmpty()) {
                Spacer(Modifier.height(24.dp))
                Text(
                    "No saved maps yet. Explore a floor, then tap Save.",
                    fontSize = Ts.Body,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    modifier = Modifier.padding(horizontal = Sp.Md),
                )
            }
            LazyColumn(
                Modifier.fillMaxSize().padding(Sp.Md),
                verticalArrangement = Arrangement.spacedBy(10.dp),
            ) {
                items(maps, key = { it.id }) { m ->
                    Card(
                        shape = RoundedCornerShape(Rd.M),
                        colors = CardDefaults.cardColors(
                            containerColor = MaterialTheme.colorScheme
                                .surfaceVariant,
                        ),
                        modifier = Modifier.fillMaxWidth().clickable {
                            // Load: base layer + nav costmap on this floor.
                            val snap = MapStore.load(ctx, m.id)
                            if (snap != null) {
                                RobotService.nav?.setMap(snap)
                                status = "Loaded ${m.name}"
                            } else {
                                status = "Load failed"
                            }
                        },
                    ) {
                        Column(Modifier.padding(Sp.Md)) {
                            Row(
                                Modifier.fillMaxWidth(),
                                horizontalArrangement =
                                    Arrangement.SpaceBetween,
                            ) {
                                Text(
                                    m.name, fontSize = Ts.Title,
                                    fontWeight = FontWeight.SemiBold,
                                )
                                Text(
                                    "Floor ${m.floor}", fontSize = Ts.Body,
                                    color = MaterialTheme.colorScheme
                                        .onSurfaceVariant,
                                )
                            }
                            Text(
                                "${m.width}×${m.height} · " +
                                    fmt.format(Date(m.savedAtMs)),
                                fontSize = Ts.Caption,
                                color = MaterialTheme.colorScheme
                                    .onSurfaceVariant,
                            )
                            Row {
                                TextButton(onClick = {
                                    editing = m
                                    newName = m.name
                                }) { Text("Rename", fontSize = Ts.Body) }
                                TextButton(onClick = {
                                    // Export JSON + PNG, share both.
                                    val snap =
                                        MapStore.load(ctx, m.id, activate = false)
                                    if (snap != null) {
                                        MapExport.export(ctx, m, snap)
                                        MapExport.shareIntent(ctx, m)?.let {
                                            ctx.startActivity(it)
                                        }
                                    }
                                }) { Text("Export", fontSize = Ts.Body) }
                                TextButton(onClick = {
                                    MapStore.delete(ctx, m.id)
                                    maps.remove(m)
                                    status = "Deleted ${m.name}"
                                }) { Text("Delete", fontSize = Ts.Body) }
                            }
                        }
                    }
                }
            }
        }
    }

    if (editing != null) {
        AlertDialog(
            onDismissRequest = { editing = null },
            title = { Text("Rename map") },
            text = {
                OutlinedTextField(
                    value = newName,
                    onValueChange = { newName = it },
                    singleLine = true,
                    label = { Text("Name") },
                )
            },
            confirmButton = {
                TextButton(onClick = {
                    editing?.let {
                        MapStore.rename(ctx, it.id, newName)
                        val i = maps.indexOfFirst { m -> m.id == it.id }
                        if (i >= 0) maps[i] = it.copy(name = newName)
                    }
                    editing = null
                }) { Text("Save") }
            },
            dismissButton = {
                TextButton(onClick = { editing = null }) { Text("Cancel") }
            },
        )
    }
}
