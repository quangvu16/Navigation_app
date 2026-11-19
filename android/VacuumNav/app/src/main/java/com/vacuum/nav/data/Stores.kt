package com.vacuum.nav.data

import android.content.Context
import org.json.JSONArray
import org.json.JSONObject
import java.io.File

/**
 * Lightweight JSON-file persistence for the UX features that land before
 * the Room database (history/schedules move to Room in Phase 8 per the
 * spec; these files keep the screens alive meanwhile). World coordinates
 * only - metres, never pixels (prompt §6).
 */

// ---------------------------------------------------------------- no-go zones
/** Axis-aligned no-go rectangle in world metres. */
data class NoGoZone(val id: Long, val x: Double, val y: Double,
                    val w: Double, val h: Double)

object ZoneStore {
    private const val FILE = "nogo_zones.json"

    fun load(ctx: Context): List<NoGoZone> = read(ctx).let { root ->
        (0 until root.length()).map { i ->
            val o = root.getJSONObject(i)
            NoGoZone(o.getLong("id"), o.getDouble("x"), o.getDouble("y"),
                o.getDouble("w"), o.getDouble("h"))
        }
    }

    fun save(ctx: Context, zones: List<NoGoZone>) {
        val arr = JSONArray()
        zones.forEach { z ->
            arr.put(JSONObject().apply {
                put("id", z.id); put("x", z.x); put("y", z.y)
                put("w", z.w); put("h", z.h)
            })
        }
        file(ctx).writeText(arr.toString())
    }

    private fun file(ctx: Context) = File(ctx.filesDir, FILE)
    private fun read(ctx: Context): JSONArray =
        if (file(ctx).exists()) JSONArray(file(ctx).readText()) else JSONArray()
}

// ------------------------------------------------------------------ schedules
/**
 * One scheduled clean: time of day, repeat days (bitmask, bit0 = Monday),
 * what to clean, enabled. Mirrors the Roborock "Scheduled Cleaning" rows
 * (time + days + target + toggle) from the reference UX.
 */
data class Schedule(
    val id: Long,
    val hour: Int,
    val minute: Int,
    val daysMask: Int,      // bit0=Mon ... bit6=Sun; 0 = once (today)
    val label: String,      // e.g. "Full Cleaning", "Bedroom"
    val enabled: Boolean,
    val group: String = "cleaning",  // "cleaning" | "routine"
) {
    val timeText: String get() = "%02d:%02d".format(hour, minute)
    val daysText: String
        get() = if (daysMask == 0) "Once"
        else listOf("Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun")
            .filterIndexed { i, _ -> daysMask and (1 shl i) != 0 }
            .joinToString(" / ")
}

object ScheduleStore {
    private const val FILE = "schedules.json"

    fun load(ctx: Context): List<Schedule> {
        val f = File(ctx.filesDir, FILE)
        if (!f.exists()) return defaultSchedules()
        val arr = JSONArray(f.readText())
        return (0 until arr.length()).map { i ->
            val o = arr.getJSONObject(i)
            Schedule(o.getLong("id"), o.getInt("hour"), o.getInt("minute"),
                o.getInt("days"), o.getString("label"),
                o.getBoolean("enabled"), o.optString("group", "cleaning"))
        }
    }

    fun save(ctx: Context, schedules: List<Schedule>) {
        val arr = JSONArray()
        schedules.forEach { s ->
            arr.put(JSONObject().apply {
                put("id", s.id); put("hour", s.hour); put("minute", s.minute)
                put("days", s.daysMask); put("label", s.label)
                put("enabled", s.enabled); put("group", s.group)
            })
        }
        File(ctx.filesDir, FILE).writeText(arr.toString())
    }

    /** First-run content matching the reference screen's shape. */
    private fun defaultSchedules() = listOf(
        Schedule(1, 10, 0, 0b0011111, "Full / Home / Dust / Mop", true),
        Schedule(2, 16, 30, 0b0010000, "Bedroom / Mop / Internet", true),
        Schedule(3, 19, 0, 0b0010000, "After Meals", false, "routine"),
        Schedule(4, 20, 30, 0b0000001, "Full Cleaning", true, "routine"),
    )
}

// ------------------------------------------------------------------ run stats
/** Live cleaning statistics for the map header (reference: Clean Area /
 *  Clean Times / Clean Time). Area accumulates path length x brush width. */
object RunStats {
    @Volatile var areaM2 = 0.0
        private set
    @Volatile var runs = 0
        private set
    @Volatile var activeSeconds = 0.0
        private set
    @Volatile var running = false
        private set
    private var lastX: Double? = null
    private var lastY: Double? = null

    fun startRun() {
        if (!running) {
            running = true
            runs += 1
            lastX = null
            lastY = null
        }
    }

    fun stopRun() {
        running = false
        lastX = null
        lastY = null
    }

    /** Advance the cleaning clock (called at the control rate). */
    fun tickRunning(dtSeconds: Double) {
        if (running) activeSeconds += dtSeconds
    }

    /** Feed the current robot pose; integrates driven distance into area. */
    fun onPose(x: Double, y: Double, brushWidthM: Double) {
        val lx = lastX
        val ly = lastY
        lastX = x
        lastY = y
        if (lx != null && ly != null && running) {
            val d = Math.hypot(x - lx, y - ly)
            areaM2 += d * brushWidthM
        }
    }

    fun areaText() = "%.0f m²".format(areaM2)
    fun timeText() = "%d min".format((activeSeconds / 60.0).toLong())
}
