package com.vacuum.nav.ui

import com.vacuum.nav.ui.theme.Rd
import com.vacuum.nav.ui.theme.Sp
import com.vacuum.nav.ui.theme.Ts

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
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.Text
import androidx.compose.material3.TopAppBar
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.ArrowBack
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.material3.AlertDialog
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.vacuum.nav.data.HistoryStore
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * Phase 8: cleaning history (prompt §6: runs with time, duration, area,
 * coverage %). JSON-file store for now; Room is the production target
 * (schema sketched in HistoryStore).
 */
// TopAppBar is still marked experimental in this material3 build.
@OptIn(androidx.compose.material3.ExperimentalMaterial3Api::class)
@Composable
fun HistoryScreen(onBack: () -> Unit, modifier: Modifier = Modifier) {
    val ctx = LocalContext.current
    val runs = remember {
        mutableStateListOf<com.vacuum.nav.data.RunRecord>().apply {
            addAll(HistoryStore.load(ctx))
        }
    }
    val fmt = remember { SimpleDateFormat("MMM d, HH:mm", Locale.US) }
    // FINDING-014: destructive actions confirm first.
    var confirmClear by remember { mutableStateOf(false) }

    Scaffold(modifier = modifier) { padding ->
        Column(Modifier.padding(padding).fillMaxSize()) {
            Row(
                Modifier.fillMaxWidth().padding(horizontal = 8.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                TopAppBar(
                title = { Text("History", fontSize = Ts.Display, fontWeight = FontWeight.Bold) },
                navigationIcon = {
                    IconButton(onClick = onBack) {
                        Icon(Icons.AutoMirrored.Filled.ArrowBack, contentDescription = "Back")
                    }
                },
            )
                Spacer(Modifier.weight(1f))
                if (runs.isNotEmpty()) {
                    TextButton(onClick = { confirmClear = true }) {
                        Text("Clear", fontSize = Ts.Body)
                    }
                }
            }
            if (runs.isEmpty()) {
                Spacer(Modifier.height(24.dp))
                Text(
                    "No runs yet. Start a clean and it will show up here.",
                    fontSize = Ts.Body,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    modifier = Modifier.padding(horizontal = Sp.Md),
                )
            }
            LazyColumn(
                Modifier.fillMaxSize().padding(Sp.Md),
                verticalArrangement = Arrangement.spacedBy(10.dp),
            ) {
                items(runs, key = { it.id }) { r ->
                    Card(
                        shape = RoundedCornerShape(Rd.M),
                        colors = CardDefaults.cardColors(
                            containerColor = MaterialTheme.colorScheme
                                .surfaceVariant,
                        ),
                        modifier = Modifier.fillMaxWidth(),
                    ) {
                        Row(
                            Modifier.fillMaxWidth().padding(Sp.Md),
                            horizontalArrangement =
                                Arrangement.SpaceBetween,
                        ) {
                            Column {
                                Text(
                                    fmt.format(Date(r.startedAtMs)),
                                    fontSize = Ts.Body,
                                    fontWeight = FontWeight.SemiBold,
                                )
                                Text(
                                    "${r.durationSec / 60} min · " +
                                        "%.1f m²".format(r.areaM2) +
                                        if (r.coveragePct > 0)
                                            " · %.0f%%".format(r.coveragePct)
                                        else "",
                                    fontSize = Ts.Caption,
                                    color = MaterialTheme.colorScheme
                                        .onSurfaceVariant,
                                )
                            }
                            Text(
                                r.mode, fontSize = Ts.Caption,
                                color = MaterialTheme.colorScheme.primary,
                            )
                        }
                    }
                }
            }
        }
    }

    if (confirmClear) {
        AlertDialog(
            onDismissRequest = { confirmClear = false },
            title = { Text("Clear history?") },
            text = { Text("All ${runs.size} past runs will be removed. This cannot be undone.") },
            confirmButton = {
                TextButton(onClick = {
                    HistoryStore.clear(ctx)
                    runs.clear()
                    confirmClear = false
                }) { Text("Clear") }
            },
            dismissButton = {
                TextButton(onClick = { confirmClear = false }) { Text("Cancel") }
            },
        )
    }
}
