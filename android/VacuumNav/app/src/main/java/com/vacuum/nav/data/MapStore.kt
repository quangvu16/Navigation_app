package com.vacuum.nav.data

import android.content.Context
import com.vacuum.nav.robot.GridSnapshot
import org.json.JSONObject
import java.io.File

/**
 * Phase 8: multi-floor map persistence (prompt §6: one map per floor;
 * maps/<id>/grid.bin RLE + meta.json). This store holds the SNAPSHOT the
 * mapper produced (grid + metadata). Submaps/pose-graph binary dumps need
 * the C++ Mapper internals and land with the SLAM polish pass - loading a
 * map here gives rendering + navigation (costmap/segmentation/frontiers)
 * on that floor immediately.
 */
data class MapMeta(
    val id: String,
    val name: String,
    val floor: Int,
    val version: Int,          // mapper keyframe version at save time
    val savedAtMs: Long,
    val width: Int,
    val height: Int,
    val originX: Int,
    val originY: Int,
    val resolution: Float,
)

object MapStore {

    /** The snapshot currently shown as the base layer (loaded from disk). */
    @Volatile
    var current: GridSnapshot? = null
        private set

    private fun root(ctx: Context) = File(ctx.filesDir, "maps").apply { mkdirs() }

    fun list(ctx: Context): List<MapMeta> =
        root(ctx).listFiles { f -> f.isDirectory }?.mapNotNull { dir ->
            runCatching {
                val o = JSONObject(File(dir, "meta.json").readText())
                MapMeta(
                    id = dir.name,
                    name = o.getString("name"),
                    floor = o.getInt("floor"),
                    version = o.getInt("version"),
                    savedAtMs = o.getLong("savedAtMs"),
                    width = o.getInt("width"),
                    height = o.getInt("height"),
                    originX = o.getInt("originX"),
                    originY = o.getInt("originY"),
                    resolution = o.getDouble("resolution").toFloat(),
                )
            }.getOrNull()
        }?.sortedBy { it.savedAtMs } ?: emptyList()

    /** Save a snapshot as a floor map; returns its metadata. */
    fun save(ctx: Context, name: String, floor: Int,
             snap: GridSnapshot): MapMeta {
        val id = "map_" + System.currentTimeMillis()
        val dir = File(root(ctx), id).apply { mkdirs() }
        // grid.bin: RLE pairs (count u16 LE, byte) over row-0-first cells.
        File(dir, "grid.bin").writeBytes(rleEncode(snap.cells))
        val meta = MapMeta(
            id = id, name = name, floor = floor, version = snap.version,
            savedAtMs = System.currentTimeMillis(),
            width = snap.width, height = snap.height,
            originX = snap.originX, originY = snap.originY,
            resolution = snap.resolution,
        )
        File(dir, "meta.json").writeText(JSONObject().apply {
            put("name", meta.name)
            put("floor", meta.floor)
            put("version", meta.version)
            put("savedAtMs", meta.savedAtMs)
            put("width", meta.width)
            put("height", meta.height)
            put("originX", meta.originX)
            put("originY", meta.originY)
            put("resolution", meta.resolution.toDouble())
        }.toString())
        return meta
    }

    /** Load a map's grid; sets [current] so the UI shows it as base layer. */
    fun load(ctx: Context, id: String): GridSnapshot? {
        val dir = File(root(ctx), id)
        val metaFile = File(dir, "meta.json")
        val gridFile = File(dir, "grid.bin")
        if (!metaFile.exists() || !gridFile.exists()) return null
        return runCatching {
            val o = JSONObject(metaFile.readText())
            val cells = rleDecode(gridFile.readBytes(),
                o.getInt("width") * o.getInt("height")) ?: return null
            GridSnapshot(
                width = o.getInt("width"),
                height = o.getInt("height"),
                originX = o.getInt("originX"),
                originY = o.getInt("originY"),
                resolution = o.getDouble("resolution").toFloat(),
                cells = cells,
                version = o.getInt("version"),
            ).also { current = it }
        }.getOrNull()
    }

    fun rename(ctx: Context, id: String, name: String) {
        val f = File(File(root(ctx), id), "meta.json")
        if (!f.exists()) return
        runCatching {
            val o = JSONObject(f.readText())
            o.put("name", name)
            f.writeText(o.toString())
        }
    }

    fun delete(ctx: Context, id: String) {
        File(root(ctx), id).deleteRecursively()
        if (current != null && current?.version?.let { true } == true) {
            // Only clear the base layer if the deleted map was showing;
            // cheap identity: compare via the meta of `current` is not
            // stored, so clearing is the safe default after a delete.
            current = null
        }
    }

    /** Drop the base layer (e.g. when a fresh mapping session starts). */
    fun clearCurrent() {
        current = null
    }

    // ---- RLE: [count u16 LE][byte] pairs, row0 first ---------------------
    fun rleEncode(cells: ByteArray): ByteArray {
        val out = java.io.ByteArrayOutputStream(cells.size)
        var i = 0
        while (i < cells.size) {
            val b = cells[i]
            var j = i + 1
            while (j < cells.size && cells[j] == b && j - i < 0xFFFF) ++j
            val n = j - i
            out.write(n and 0xFF)
            out.write((n shr 8) and 0xFF)
            out.write(b.toInt() and 0xFF)
            i = j
        }
        return out.toByteArray()
    }

    fun rleDecode(data: ByteArray, expected: Int): ByteArray? {
        val out = ByteArray(expected)
        var p = 0
        var i = 0
        while (p + 2 < data.size && i < expected) {
            val n = (data[p].toInt() and 0xFF) or
                ((data[p + 1].toInt() and 0xFF) shl 8)
            val b = data[p + 2]
            p += 3
            if (i + n > expected) return null
            java.util.Arrays.fill(out, i, i + n, b)
            i += n
        }
        return if (i == expected) out else null
    }
}
