package com.vacuum.nav.robot

import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/** Go-to state codes mirrored from C++ GoToState (goto_controller.h). */
object NavState {
    const val IDLE = 0
    const val REJECTED = 1   // goal in lethal - show "invalid goal"
    const val PLANNING = 2
    const val FOLLOWING = 3
    const val RECOVERING = 4
    const val REACHED = 5
    const val STUCK = 6      // gave up after3 recoveries

    fun label(code: Int): String = when (code) {
        REJECTED -> "Invalid goal"
        PLANNING -> "Planning…"
        FOLLOWING -> "Driving to goal"
        RECOVERING -> "Unsticking…"
        REACHED -> "Goal reached"
        STUCK -> "Stuck — gave up"
        else -> ""
    }
}

data class NavUiState(
    val state: Int = NavState.IDLE,
    val goal: Pair<Double, Double>? = null,
    val path: FloatArray = FloatArray(0),   // flattened x,y pairs
    val pathVersion: Int = 0,
) {
    val active: Boolean
        get() = state == NavState.PLANNING || state == NavState.FOLLOWING ||
            state == NavState.RECOVERING
    val finished: Boolean
        get() = state == NavState.REACHED || state == NavState.STUCK ||
            state == NavState.REJECTED

    override fun equals(other: Any?): Boolean =
        other is NavUiState && other.state == state && other.goal == goal &&
            other.pathVersion == pathVersion
    override fun hashCode(): Int = state * 31 + pathVersion
}

/**
 * Phase 4 go-to-goal: JNI wrapper over vacuum_core's GoToController with a
 * layered costmap (static map snapshot + no-go keepouts + decaying scan
 * obstacles). The service steps it at 20 Hz with the current pose; the map
 * screen draws goal + path from [state].
 */
class NavEngine(private val config: RobotConfig) : AutoCloseable {

    private var handle: Long = nativeCreate(config.toJson())

    private val _state = MutableStateFlow(NavUiState())
    val state: StateFlow<NavUiState> = _state.asStateFlow()

    @Volatile
    private var pathVersion = 0

    /** Install the static layer from a MapperEngine GridSnapshot. */
    fun setMap(snapshot: ByteArray): Boolean {
        if (handle == 0L) return false
        return nativeSetMap(handle, snapshot)
    }

    /** Convenience: re-serialise a GridSnapshot into the native format. */
    fun setMap(s: GridSnapshot): Boolean {
        val out = ByteArray(20 + s.cells.size)
        fun putI32(off: Int, v: Int) {
            out[off] = (v and 0xFF).toByte()
            out[off + 1] = ((v shr 8) and 0xFF).toByte()
            out[off + 2] = ((v shr 16) and 0xFF).toByte()
            out[off + 3] = ((v shr 24) and 0xFF).toByte()
        }
        putI32(0, s.width)
        putI32(4, s.height)
        putI32(8, s.originX)
        putI32(12, s.originY)
        val bits = s.resolution.toRawBits()
        out[16] = (bits and 0xFF).toByte()
        out[17] = ((bits shr 8) and 0xFF).toByte()
        out[18] = ((bits shr 16) and 0xFF).toByte()
        out[19] = ((bits shr 24) and 0xFF).toByte()
        s.cells.copyInto(out, 20)
        return setMap(out)
    }

    /** No-go rectangles (x, y, w, h) in world metres. */
    fun setKeepouts(rects: FloatArray) {
        if (handle == 0L) return
        nativeSetKeepouts(handle, rects)
    }

    /** Live obstacle points from the latest scan (world metres). */
    fun setObstacles(xy: FloatArray, count: Int) {
        if (handle == 0L) return
        nativeSetObstacles(handle, xy, count)
    }

    /** Returns false (and latches REJECTED) when the goal is in lethal. */
    fun setGoal(x: Double, y: Double): Boolean {
        if (handle == 0L) return false
        val ok = nativeSetGoal(handle, x.toFloat(), y.toFloat()) == 0
        _state.value = _state.value.copy(
            state = if (ok) NavState.PLANNING else NavState.REJECTED,
            goal = if (ok) (x to y) else null,
        )
        return ok
    }

    fun cancel() {
        if (handle == 0L) return
        nativeCancel(handle)
        _state.value = NavUiState()
    }

    /**
     * One control tick (20 Hz). Returns (v m/s, w rad/s) for the caller to
     * convert to wheel RPM; publishes state + path for the UI.
     */
    fun step(poseX: Float, poseY: Float, poseTheta: Float): Pair<Double, Double> {
        if (handle == 0L) return 0.0 to 0.0
        val out = nativeStep(handle, poseX, poseY, poseTheta)
        val code = out[2].toInt()
        val path = if (code != NavState.IDLE && code != _state.value.state) {
            nativePath(handle).also { pathVersion++ }
        } else _state.value.path
        _state.value = _state.value.copy(state = code, path = path,
            pathVersion = pathVersion)
        return out[0].toDouble() to out[1].toDouble()
    }

    override fun close() {
        if (handle != 0L) {
            nativeDestroy(handle)
            handle = 0L
        }
    }

    /** Phase 6: best frontier goal from (x, y), or null when exploration
     *  is finished (no reachable frontier > 0.5 m). */
    fun exploreGoal(x: Float, y: Float): Pair<Double, Double>? {
        if (handle == 0L) return null
        val g = nativeExploreGoal(handle, x, y) ?: return null
        return g[0].toDouble() to g[1].toDouble()
    }

    private external fun nativeExploreGoal(handle: Long, px: Float,
                                           py: Float): FloatArray?

    // ---------------- Phase 7: room segmentation --------------------------
    data class RoomInfo(val id: Int, val areaM2: Float,
                        val cx: Float, val cy: Float)

    data class SegmentResult(
        val width: Int, val height: Int,
        val originX: Int, val originY: Int,
        val resolution: Float,
        val rooms: List<RoomInfo>,
        val labels: ByteArray,   // row 0 = world bottom; 0 = not a room
    )

    /** Segment the current map (cached in C++ until the map changes). */
    fun segment(): SegmentResult? {
        if (handle == 0L) return null
        val raw = nativeSegment(handle) ?: return null
        if (raw.size < 21) return null
        fun i32(off: Int): Int =
            (raw[off].toInt() and 0xFF) or
                ((raw[off + 1].toInt() and 0xFF) shl 8) or
                ((raw[off + 2].toInt() and 0xFF) shl 16) or
                ((raw[off + 3].toInt() and 0xFF) shl 24)
        val w = i32(0); val h = i32(4)
        val gx0 = i32(8); val gy0 = i32(12)
        val resBits = (raw[16].toInt() and 0xFF) or
            ((raw[17].toInt() and 0xFF) shl 8) or
            ((raw[18].toInt() and 0xFF) shl 16) or
            ((raw[19].toInt() and 0xFF) shl 24)
        val res = Float.fromBits(resBits)
        val nRooms = raw[20].toInt() and 0xFF
        var o = 21
        val rooms = ArrayList<RoomInfo>(nRooms)
        repeat(nRooms) {
            val id = raw[o].toInt() and 0xFF
            val area = Float.fromBits(
                (raw[o + 1].toInt() and 0xFF) or
                    ((raw[o + 2].toInt() and 0xFF) shl 8) or
                    ((raw[o + 3].toInt() and 0xFF) shl 16) or
                    ((raw[o + 4].toInt() and 0xFF) shl 24))
            val cx = Float.fromBits(
                (raw[o + 5].toInt() and 0xFF) or
                    ((raw[o + 6].toInt() and 0xFF) shl 8) or
                    ((raw[o + 7].toInt() and 0xFF) shl 16) or
                    ((raw[o + 8].toInt() and 0xFF) shl 24))
            val cy = Float.fromBits(
                (raw[o + 9].toInt() and 0xFF) or
                    ((raw[o + 10].toInt() and 0xFF) shl 8) or
                    ((raw[o + 11].toInt() and 0xFF) shl 16) or
                    ((raw[o + 12].toInt() and 0xFF) shl 24))
            rooms.add(RoomInfo(id, area, cx, cy))
            o += 13
        }
        val labels = raw.copyOfRange(o, o + w * h)
        return SegmentResult(w, h, gx0, gy0, res, rooms, labels)
    }

    private external fun nativeSegment(handle: Long): ByteArray?

    private external fun nativeCreate(configJson: String): Long
    private external fun nativeDestroy(handle: Long)
    private external fun nativeSetMap(handle: Long, snapshot: ByteArray): Boolean
    private external fun nativeSetKeepouts(handle: Long, rects: FloatArray)
    private external fun nativeSetObstacles(handle: Long, xy: FloatArray, count: Int)
    private external fun nativeSetGoal(handle: Long, x: Float, y: Float): Int
    private external fun nativeCancel(handle: Long)
    private external fun nativeStep(handle: Long, px: Float, py: Float,
                                    pth: Float): FloatArray
    private external fun nativePath(handle: Long): FloatArray

    companion object {
        init {
            System.loadLibrary("vacuum_jni")
        }
    }
}
