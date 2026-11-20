package com.vacuum.nav.data

import android.content.Context
import org.json.JSONObject
import java.io.File

/**
 * Phase 7 polish: user room names keyed by the segmenter's STABLE ids
 * (prompt §5: "user renames/merges/splits are stored and reapplied").
 * Renames survive resegmentation as long as the id survives; merges are
 * applied to the label grid itself (NavEngine.nativeMergeRooms), so only
 * names of surviving ids matter.
 */
object RoomNames {
    private fun file(ctx: Context) = File(ctx.filesDir, "room_names.json")

    fun all(ctx: Context): Map<Int, String> {
        val f = file(ctx)
        if (!f.exists()) return emptyMap()
        return runCatching {
            val o = JSONObject(f.readText())
            val out = HashMap<Int, String>()
            for (k in o.keys()) out[k.toInt()] = o.getString(k)
            out
        }.getOrDefault(emptyMap())
    }

    fun get(ctx: Context, id: Int): String = all(ctx)[id] ?: "Room $id"

    fun set(ctx: Context, id: Int, name: String) {
        val o = JSONObject(all(ctx).mapKeys { it.key.toString() }
            .mapValues { it.value })
        o.put(id.toString(), name)
        file(ctx).writeText(o.toString())
    }
}
