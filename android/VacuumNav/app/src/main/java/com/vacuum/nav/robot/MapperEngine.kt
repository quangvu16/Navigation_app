package com.vacuum.nav.robot

import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/** Occupancy snapshot from the C++ Mapper (row 0 = world bottom). */
data class GridSnapshot(
    val width: Int,
    val height: Int,
    val originX: Int,     // cell coords of column 0
    val originY: Int,     // cell coords of row 0
    val resolution: Float,
    val cells: ByteArray, // 0 free, 1 occupied, 2 unknown
    val version: Int,
) {
    override fun equals(other: Any?): Boolean =
        other is GridSnapshot && other.version == version
    override fun hashCode(): Int = version
}

/**
 * Phase 3 live mapping: JNI wrapper over vacuum_core's Mapper (correlative
 * scan matching + keyframes + loop closure, host-tested in
 * test_mapping.cpp). The service feeds every scan with the current pose
 * guess; the map screen rebuilds its bitmap only when [version] changes.
 */
class MapperEngine(private val config: RobotConfig) : AutoCloseable {

    private var handle: Long = nativeCreate(config.toJson())

    private val _snapshot = MutableStateFlow<GridSnapshot?>(null)
    val snapshot: StateFlow<GridSnapshot?> = _snapshot.asStateFlow()

    private val _matchedPose = MutableStateFlow(Pose.ZERO)
    val matchedPose: StateFlow<Pose> = _matchedPose.asStateFlow()

    @Volatile
    private var lastVersion = 0

    /**
     * Feed one scan (wire ranges, metres, 0 = no return) with the pose
     * guess in the map frame. Returns the scan-matched pose.
     */
    fun feed(ranges: FloatArray, guess: Pose): Pose {
        if (handle == 0L) return guess
        val out = FloatArray(3)
        nativeFeed(handle, ranges, ranges.size,
            guess.x, guess.y, guess.theta, out)
        val matched = Pose(out[0], out[1], out[2])
        _matchedPose.value = matched

        val v = nativeVersion(handle)
        if (v != lastVersion) {
            lastVersion = v
            _snapshot.value = readSnapshot(v)
        }
        return matched
    }

    /** Force a snapshot read (e.g. when the Map tab opens). */
    fun refresh() {
        if (handle == 0L) return
        _snapshot.value = readSnapshot(nativeVersion(handle))
    }

    private fun readSnapshot(version: Int): GridSnapshot? {
        val raw = nativeSnapshot(handle) ?: return null
        if (raw.size < 20) return null
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
        if (w <= 0 || h <= 0 || raw.size < 20 + w * h) return null
        return GridSnapshot(w, h, gx0, gy0, res,
            raw.copyOfRange(20, 20 + w * h), version)
    }

    override fun close() {
        if (handle != 0L) {
            nativeDestroy(handle)
            handle = 0L
        }
    }

    private external fun nativeCreate(configJson: String): Long
    private external fun nativeDestroy(handle: Long)
    private external fun nativeFeed(
        handle: Long, ranges: FloatArray, bins: Int,
        poseX: Float, poseY: Float, poseTheta: Float, outPose: FloatArray,
    )
    private external fun nativeVersion(handle: Long): Int
    private external fun nativeSnapshot(handle: Long): ByteArray?

    companion object {
        init {
            System.loadLibrary("vacuum_jni")
        }
    }
}
