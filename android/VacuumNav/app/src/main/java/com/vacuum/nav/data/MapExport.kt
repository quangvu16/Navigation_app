package com.vacuum.nav.data

import android.content.Context
import android.content.Intent
import android.graphics.Bitmap
import androidx.core.content.FileProvider
import com.vacuum.nav.robot.GridSnapshot
import org.json.JSONObject
import java.io.File
import java.util.Base64

/**
 * Phase 8: map export/import as JSON + PNG (prompt §6 screens).
 *
 * Format (documented for external consumers, e.g. Home Assistant later):
 *   { "format": "vacuumnav.map", "formatVersion": 1,
 *     "name", "floor", "version", "width", "height",
 *     "originX", "originY", "resolution",
 *     "cellsRle": base64 of [count u16 LE][byte] pairs (row0 first) }
 * The PNG is the same picture the app draws (walls dark, free light,
 * unknown transparent over white) for sharing/records.
 */
object MapExport {

    const val FORMAT = "vacuumnav.map"
    const val FORMAT_VERSION = 1

    private fun exportDir(ctx: Context): File =
        File(ctx.getExternalFilesDir(null) ?: ctx.filesDir, "exports")
            .apply { mkdirs() }

    /** Write <name>.json + <name>.png; returns the JSON file. */
    fun export(ctx: Context, meta: MapMeta, snap: GridSnapshot): File {
        val base = meta.name.replace(Regex("[^A-Za-z0-9._-]"), "_")
        val json = JSONObject().apply {
            put("format", FORMAT)
            put("formatVersion", FORMAT_VERSION)
            put("name", meta.name)
            put("floor", meta.floor)
            put("version", meta.version)
            put("width", snap.width)
            put("height", snap.height)
            put("originX", snap.originX)
            put("originY", snap.originY)
            put("resolution", snap.resolution.toDouble())
            put("cellsRle", Base64.getEncoder()
                .encodeToString(MapStore.rleEncode(snap.cells)))
        }
        val jsonFile = File(exportDir(ctx), "$base.json")
        jsonFile.writeText(json.toString())
        pngOf(snap).compress(Bitmap.CompressFormat.PNG, 100,
            File(exportDir(ctx), "$base.png").outputStream())
        return jsonFile
    }

    /** Share both files through the FileProvider (ACTION_SEND_MULTIPLE). */
    fun shareIntent(ctx: Context, meta: MapMeta): Intent? {
        val base = meta.name.replace(Regex("[^A-Za-z0-9._-]"), "_")
        val j = File(exportDir(ctx), "$base.json")
        val p = File(exportDir(ctx), "$base.png")
        if (!j.exists()) return null
        val authority = "${ctx.packageName}.fileprovider"
        val uris = ArrayList<android.net.Uri>()
        uris.add(FileProvider.getUriForFile(ctx, authority, j))
        if (p.exists()) uris.add(FileProvider.getUriForFile(ctx, authority, p))
        return Intent(Intent.ACTION_SEND_MULTIPLE).apply {
            type = "*/*"
            putParcelableArrayListExtra(Intent.EXTRA_STREAM, uris)
            addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        }
    }

    /** Import from a content URI; saves as a new floor map. */
    fun import(ctx: Context, uri: android.net.Uri): MapMeta? = runCatching {
        val text = ctx.contentResolver.openInputStream(uri)?.use {
            it.readBytes().toString(Charsets.UTF_8)
        } ?: return null
        val o = JSONObject(text)
        if (o.getString("format") != FORMAT) return null
        val cells = MapStore.rleDecode(
            Base64.getDecoder().decode(o.getString("cellsRle")),
            o.getInt("width") * o.getInt("height")) ?: return null
        val snap = GridSnapshot(
            width = o.getInt("width"),
            height = o.getInt("height"),
            originX = o.getInt("originX"),
            originY = o.getInt("originY"),
            resolution = o.getDouble("resolution").toFloat(),
            cells = cells,
            version = o.getInt("version"),
        )
        val floor = (MapStore.list(ctx).maxOfOrNull { it.floor } ?: 0) + 1
        MapStore.save(ctx, o.optString("name", "Imported") +
            " ($floor)", floor, snap)
    }.getOrNull()

    /** The app-style rendering of a snapshot as a PNG. */
    fun pngOf(snap: GridSnapshot): Bitmap {
        val px = IntArray(snap.width * snap.height)
        for (y in 0 until snap.height) {
            val row = snap.height - 1 - y
            for (x in 0 until snap.width) {
                px[row * snap.width + x] = when (
                    snap.cells[y * snap.width + x].toInt()
                ) {
                    1 -> 0xFF4A4D52.toInt()
                    0 -> 0xFFE8E6E1.toInt()
                    else -> 0xFFFFFFFF.toInt()
                }
            }
        }
        return Bitmap.createBitmap(px, snap.width, snap.height,
            Bitmap.Config.ARGB_8888)
    }
}
