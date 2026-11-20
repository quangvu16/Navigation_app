package com.vacuum.nav.ui

import com.vacuum.nav.ui.theme.*

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
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
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.FloatingActionButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Surface
import androidx.compose.material3.Switch
import androidx.compose.material3.SwitchDefaults
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.TimePicker
import androidx.compose.material3.rememberTimePickerState
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.vacuum.nav.data.Schedule
import com.vacuum.nav.data.ScheduleStore
import androidx.compose.ui.graphics.Color

/**
 * Schedules, in the shape of the reference screen: two groups (Scheduled
 * Cleaning / Scheduled Routines), each row = time + days + target + toggle,
 * FAB opens a time-picker dialog. Persisted via ScheduleStore (Room in
 * Phase 8 per the spec).
 */
@Composable
fun SchedulesScreen(modifier: Modifier = Modifier) {
    val ctx = LocalContext.current
    val schedules = remember {
        mutableStateListOf<Schedule>().apply { addAll(ScheduleStore.load(ctx)) }
    }
    var editing by remember { mutableStateOf<Schedule?>(null) }
    var adding by remember { mutableStateOf(false) }
    var tab by remember { mutableStateOf("cleaning") }

    fun persist() {
        val list = schedules.toList()
        ScheduleStore.save(ctx, list)
        // Phase 8: arm/re-arm the AlarmManager timers on every change.
        com.vacuum.nav.data.Scheduler.scheduleAll(ctx, list)
    }

    Scaffold(
        modifier = modifier,
        floatingActionButton = {
            FloatingActionButton(
                onClick = { adding = true },
                containerColor = MaterialTheme.colorScheme.primary,
            ) { Text("+", fontSize = Ts.Display,
                color = androidx.compose.ui.graphics.Color.White) }
        },
    ) { padding ->
        Column(Modifier.padding(padding).fillMaxSize()) {
            Text(
                "Schedules",
                fontSize = Ts.Display, fontWeight = FontWeight.Bold,
                modifier = Modifier.padding(start = 16.dp, top = 12.dp),
            )
            Row(
                Modifier.fillMaxWidth().padding(horizontal = Sp.Md),
                horizontalArrangement = Arrangement.spacedBy(Sp.Xs),
            ) {
                listOf("cleaning" to "Cleaning", "routine" to "Routines").forEach { (key, label) ->
                    val selected = tab == key
                    Surface(
                        onClick = { tab = key },
                        modifier = Modifier.weight(1f).height(40.dp),
                        shape = RoundedCornerShape(50),
                        color = if (selected) MaterialTheme.colorScheme.primaryContainer
                        else MaterialTheme.colorScheme.surfaceVariant,
                    ) {
                        Box(contentAlignment = Alignment.Center) {
                            Text(
                                label, fontSize = Ts.Body,
                                fontWeight = if (selected) FontWeight.Bold else FontWeight.Normal,
                                color = if (selected) MaterialTheme.colorScheme.onPrimaryContainer
                                else MaterialTheme.colorScheme.onSurfaceVariant,
                            )
                        }
                    }
                }
            }
            LazyColumn(
                Modifier.fillMaxSize().padding(Sp.Md),
                verticalArrangement = Arrangement.spacedBy(10.dp),
            ) {
                val rows = schedules.filter { it.group == tab }
                if (rows.isEmpty()) {
                    item {
                        Text(
                            if (tab == "cleaning") "No cleanings yet. Tap + to add one."
                            else "No routines yet. Tap + to add one.",
                            fontSize = Ts.Body,
                            color = MaterialTheme.colorScheme.onSurfaceVariant,
                            modifier = Modifier.padding(top = 8.dp),
                        )
                    }
                }
                items(rows, key = { it.id }) { s ->
                    ScheduleRow(
                        s,
                        onToggle = { on ->
                            val i = schedules.indexOfFirst { it.id == s.id }
                            if (i >= 0) {
                                schedules[i] = schedules[i].copy(enabled = on)
                                persist()
                            }
                        },
                        onClick = { editing = s },
                    )
                }
                item { Spacer(Modifier.height(72.dp)) }
            }
        }
    }

    if (adding || editing != null) {
        ScheduleDialog(
            initial = editing,
            onSave = { s ->
                if (editing != null) {
                    val i = schedules.indexOfFirst { it.id == s.id }
                    if (i >= 0) schedules[i] = s
                } else {
                    schedules.add(s)
                }
                persist()
                adding = false
                editing = null
            },
            onCancel = { adding = false; editing = null },
        )
    }
}

@Composable
private fun ScheduleRow(s: Schedule, onToggle: (Boolean) -> Unit,
                        onClick: () -> Unit) {
    Card(
        onClick = onClick,
        shape = RoundedCornerShape(Rd.M),
        colors = CardDefaults.cardColors(
            containerColor = MaterialTheme.colorScheme.surface
        ),
    ) {
        Row(
            Modifier.fillMaxWidth().padding(horizontal = Sp.Md, vertical = 12.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text(
                s.timeText,
                fontSize = Ts.Display, fontWeight = FontWeight.Bold,
                modifier = Modifier.padding(end = 16.dp),
            )
            Column(Modifier.weight(1f)) {
                Text(s.daysText, fontSize = Ts.Body,
                    fontWeight = FontWeight.Medium)
                Text(
                    s.label, fontSize = Ts.Caption,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
            Switch(
                checked = s.enabled,
                onCheckedChange = onToggle,
                colors = SwitchDefaults.colors(
                    checkedTrackColor = RobotOrange,
                    checkedThumbColor = Color.White,
                ),
            )
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun ScheduleDialog(initial: Schedule?, onSave: (Schedule) -> Unit,
                           onCancel: () -> Unit) {
    val timeState = rememberTimePickerState(
        initialHour = initial?.hour ?: 10,
        initialMinute = initial?.minute ?: 0,
    )
    var label by remember { mutableStateOf(initial?.label ?: "Full Cleaning") }
    var days by remember { mutableStateOf(initial?.daysMask ?: 0b0011111) }
    var group by remember { mutableStateOf(initial?.group ?: "cleaning") }

    AlertDialog(
        onDismissRequest = onCancel,
        title = { Text(if (initial == null) "New schedule" else "Edit schedule") },
        text = {
            Column {
                TimePicker(state = timeState)
                Spacer(Modifier.height(8.dp))
                Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                    listOf("Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun")
                        .forEachIndexed { i, d ->
                            val on = days and (1 shl i) != 0
                            TextButton(onClick = {
                                days = if (on) days and (1 shl i).inv()
                                else days or (1 shl i)
                            }) {
                                Text(
                                    d, fontSize = Ts.Caption,
                                    color = if (on) MaterialTheme.colorScheme
                                        .primary
                                    else MaterialTheme.colorScheme
                                        .onSurfaceVariant,
                                )
                            }
                        }
                }
                OutlinedTextField(
                    value = label,
                    onValueChange = { label = it },
                    label = { Text("Target / name") },
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth(),
                )
                Row {
                    TextButton(onClick = { group = "cleaning" }) {
                        Text("Cleaning",
                            fontWeight = if (group == "cleaning")
                                FontWeight.Bold else FontWeight.Normal)
                    }
                    TextButton(onClick = { group = "routine" }) {
                        Text("Routine",
                            fontWeight = if (group == "routine")
                                FontWeight.Bold else FontWeight.Normal)
                    }
                }
            }
        },
        confirmButton = {
            Button(onClick = {
                onSave(
                    Schedule(
                        id = initial?.id ?: System.nanoTime(),
                        hour = timeState.hour,
                        minute = timeState.minute,
                        daysMask = days,
                        label = label.ifBlank { "Full Cleaning" },
                        enabled = initial?.enabled ?: true,
                        group = group,
                    )
                )
            }) { Text("Save") }
        },
        dismissButton = { TextButton(onClick = onCancel) { Text("Cancel") } },
    )
}
