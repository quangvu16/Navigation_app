package com.vacuum.nav.data

import android.content.Context
import org.json.JSONObject

/**
 * Phase 8: dock pose saved on the map ("dock is here", prompt §5/§6).
 * World metres; the dock heading is the direction the dock FACES (the
 * robot backs in along it, per the repo's reverse-in approach - see
 * docs/ASSUMPTIONS.md §4.1).
 */
data class DockPose(val x: Double, val y: Double, val theta: Double)

object DockStore {
    private const val FILE = "dock.json"

    fun save(ctx: Context, dock: DockPose) {
        File_(ctx).writeText(JSONObject().apply {
            put("x", dock.x); put("y", dock.y); put("theta", dock.theta)
        }.toString())
    }

    fun load(ctx: Context): DockPose? {
        val f = File_(ctx)
        if (!f.exists()) return null
        return try {
            val o = JSONObject(f.readText())
            DockPose(o.getDouble("x"), o.getDouble("y"), o.getDouble("theta"))
        } catch (_: Exception) {
            null
        }
    }

    fun clear(ctx: Context) {
        File_(ctx).delete()
    }

    private fun File_(ctx: Context) = java.io.File(ctx.filesDir, FILE)
}
