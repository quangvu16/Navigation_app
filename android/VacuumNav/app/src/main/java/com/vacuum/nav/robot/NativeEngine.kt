package com.vacuum.nav.robot

/**
 * JNI surface over vacuum_core (app/src/main/cpp/jni_bridge.cpp). One
 * instance = one simulated robot. All methods block briefly on an internal
 * mutex; call them from the link's own thread, not the UI thread.
 */
class NativeEngine(
    configJson: String,
    worldCells: ByteArray?,
    worldW: Int,
    worldH: Int,
    worldRes: Double,
    originX: Double,
    originY: Double,
    seed: Long = 1L,
) : AutoCloseable {

    private var handle: Long = nativeCreate(
        configJson, worldCells, worldW, worldH, worldRes, originX, originY, seed
    )

    fun setWheelRpm(rpmL: Float, rpmR: Float) = nativeSetWheelRpm(handle, rpmL, rpmR)
    fun step(dt: Double) = nativeStep(handle, dt)

    // JNI getters fill an out-array (no allocation on the native side); the
    // wrappers hand the filled array back to Kotlin callers.
    fun pose(): FloatArray = FloatArray(3).also { nativeGetPose(handle, it) }
    fun odom(): FloatArray = FloatArray(3).also { nativeGetOdom(handle, it) }

    fun scanRanges(out: FloatArray): FloatArray = out.also { nativeGetScanRanges(handle, it) }
    fun scanIntensity(out: ByteArray): ByteArray = out.also { nativeGetScanIntensity(handle, it) }

    /** [scanId, startS, endS, rotationHz] */
    fun scanMeta(): FloatArray = FloatArray(4).also { nativeGetScanMeta(handle, it) }

    /** [front, left, right] metres, 0 = no echo. */
    fun sonar(): FloatArray = FloatArray(3).also { nativeGetSonar(handle, it) }

    /** [gyroZ, batteryV, rpmL, rpmR, ticksL, ticksR] */
    fun status(): FloatArray = FloatArray(6).also { nativeGetStatus(handle, it) }

    override fun close() {
        if (handle != 0L) {
            nativeDestroy(handle)
            handle = 0L
        }
    }

    private external fun nativeCreate(
        configJson: String, worldCells: ByteArray?, worldW: Int, worldH: Int,
        worldRes: Double, originX: Double, originY: Double, seed: Long,
    ): Long

    private external fun nativeDestroy(handle: Long)
    private external fun nativeSetWheelRpm(handle: Long, rpmL: Float, rpmR: Float)
    private external fun nativeStep(handle: Long, dt: Double)
    private external fun nativeGetPose(handle: Long, out3: FloatArray)
    private external fun nativeGetOdom(handle: Long, out3: FloatArray)
    private external fun nativeGetScanRanges(handle: Long, out: FloatArray)
    private external fun nativeGetScanIntensity(handle: Long, out: ByteArray)
    private external fun nativeGetScanMeta(handle: Long, out4: FloatArray)
    private external fun nativeGetSonar(handle: Long, out3: FloatArray)
    private external fun nativeGetStatus(handle: Long, out6: FloatArray)

    companion object {
        init {
            System.loadLibrary("vacuum_jni")
        }
    }
}
