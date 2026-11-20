package com.vacuum.nav.ui

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
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.remember
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
@Composable
fun HistoryScreen(onBack: () -> Unit, modifier: Modifier = Modifier) {
    val ctx = LocalContext.current
    val runs = remember {
        mutableStateListOf<com.vacuum.nav.data.RunRecord>().apply {
            addAll(HistoryStore.load(ctx))
        }
    }
    val fmt = remember { SimpleDateFormat("MMM d, HH:mm", Locale.US) }

    Scaffold(modifier = modifier) { padding ->
        Column(Modifier.padding(padding).fillMaxSize()) {
            Row(
                Modifier.fillMaxWidth().padding(horizontal = 8.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                TextButton(onClick = onBack) { Text("‹ Back") }
                Text("History", fontSize = 22.sp,
                    fontWeight = FontWeight.Bold)
                Spacer(Modifier.weight(1f))
                if (runs.isNotEmpty()) {
                    TextButton(onClick = {
                        HistoryStore.clear(ctx)
                        runs.clear()
                    }) { Text("Clear", fontSize = 13.sp) }
                }
            }
            if (runs.isEmpty()) {
                Spacer(Modifier.height(24.dp))
                Text(
                    "No runs yet — start a clean and it will show up here.",
                    fontSize = 14.sp,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    modifier = Modifier.padding(horizontal = 16.dp),
                )
            }
            LazyColumn(
                Modifier.fillMaxSize().padding(16.dp),
                verticalArrangement = Arrangement.spacedBy(10.dp),
            ) {
                items(runs, key = { it.id }) { r ->
                    Card(
                        shape = RoundedCornerShape(16.dp),
                        colors = CardDefaults.cardColors(
                            containerColor = MaterialTheme.colorScheme
                                .surfaceVariant,
                        ),
                        modifier = Modifier.fillMaxWidth(),
                    ) {
                        Row(
                            Modifier.fillMaxWidth().padding(16.dp),
                            horizontalArrangement =
                                Arrangement.SpaceBetween,
                        ) {
                            Column {
                                Text(
                                    fmt.format(Date(r.startedAtMs)),
                                    fontSize = 15.sp,
                                    fontWeight = FontWeight.SemiBold,
                                )
                                Text(
                                    "${r.durationSec / 60} min · " +
                                        "%.1f m²".format(r.areaM2) +
                                        if (r.coveragePct > 0)
                                            " · %.0f%%".format(r.coveragePct)
                                        else "",
                                    fontSize = 12.sp,
                                    color = MaterialTheme.colorScheme
                                        .onSurfaceVariant,
                                )
                            }
                            Text(
                                r.mode, fontSize = 12.sp,
                                color = MaterialTheme.colorScheme.primary,
                            )
                        }
                    }
                }
            }
        }
    }
}
