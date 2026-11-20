package com.vacuum.nav.data

import android.content.Context
import org.json.JSONObject
import java.io.File

/**
 * Phase 8: cleaning history (prompt §6 screens). JSON-file persistence
 * with the Room schema sketched below - the production target is Room
 * (the spec's choice); this keeps the screen honest until the KSP/Room
 * migration lands without risking the AGP 9 build.
 *
 * Room entity (planned):
 *   @Entity data class RunEntity(
 *     @PrimaryKey val id: Long, val startedAt: Long, val durationSec: Long,
 *     val areaM2: Double, val coveragePct: Double, val mode: String)
 */
data class RunRecord(
    val id: Long,
    val startedAtMs: Long,
    val durationSec: Long,
    val areaM2: Double,
    val coveragePct: Double,   //0 until the Phase-7 tracker feeds it
    val mode: String,          // "full" | "rooms" | "zones" | "explore"
)

object HistoryStore {
    private const val MAX = 200

    private fun file(ctx: Context) = File(ctx.filesDir, "history.json")

    fun load(ctx: Context): List<RunRecord> {
        val f = file(ctx)
        if (!f.exists()) return emptyList()
        return runCatching {
            val arr = org.json.JSONArray(f.readText())
            (0 until arr.length()).map { i ->
                val o = arr.getJSONObject(i)
                RunRecord(
                    id = o.getLong("id"),
                    startedAtMs = o.getLong("startedAt"),
                    durationSec = o.getLong("durationSec"),
                    areaM2 = o.getDouble("areaM2"),
                    coveragePct = o.getDouble("coveragePct"),
                    mode = o.optString("mode", "full"),
                )
            }
        }.getOrDefault(emptyList())
    }

    fun add(ctx: Context, run: RunRecord) {
        val all = (listOf(run) + load(ctx)).take(MAX)
        val arr = org.json.JSONArray()
        all.forEach { r ->
            arr.put(JSONObject().apply {
                put("id", r.id)
                put("startedAt", r.startedAtMs)
                put("durationSec", r.durationSec)
                put("areaM2", r.areaM2)
                put("coveragePct", r.coveragePct)
                put("mode", r.mode)
            })
        }
        file(ctx).writeText(arr.toString())
    }

    fun clear(ctx: Context) {
        file(ctx).delete()
    }
}
