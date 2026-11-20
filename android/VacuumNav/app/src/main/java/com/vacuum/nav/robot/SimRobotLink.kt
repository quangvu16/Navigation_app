package com.vacuum.nav.robot

import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import java.util.concurrent.atomic.AtomicBoolean

/**
 * RobotLink backed by the vacuum_core simulator. Owns one thread that steps
 * physics at a fixed rate and publishes telemetry at 60 Hz — exactly the
 * cadence the map canvas consumes, so the UI never blocks on physics.
 *
 * Later phases swap this for UdpRobotLink; the flows stay identical.
 */
class SimRobotLink(
    private val config: RobotConfig = RobotConfig(),
    seed: Long = 1L,
) : RobotLink {
    override val description: String get() = "SIM · simulator"


    private val engine = NativeEngine(config.toJson(), null, 0, 0, 0.0, 0.0, 0.0, seed)

    private val _telemetry = MutableStateFlow(Telemetry())
    override val telemetry: StateFlow<Telemetry> = _telemetry.asStateFlow()

    private val _scans = MutableSharedFlow<Scan>(replay = 1, extraBufferCapacity = 4)
    override val scans: SharedFlow<Scan> = _scans.asSharedFlow()

    private val running = AtomicBoolean(true)
    private val thread = Thread(::loop, "sim-robot-link").apply {
        isDaemon = true
        start()
    }

    @Volatile
    private var rpmL = 0f

    @Volatile
    private var rpmR = 0f

    override fun setWheelRpm(rpmL: Float, rpmR: Float) {
        this.rpmL = rpmL
        this.rpmR = rpmR
        engine.setWheelRpm(rpmL, rpmR)
    }

    override fun close() {
        running.set(false)
        thread.join(500)
        engine.close()
    }

    private fun loop() {
        // Fixed 200 Hz physics; 60 Hz telemetry publication. The scan array
        // is only re-read when scanId moves (a revolution completed).
        val dt = 0.005
        val publishEvery = 3 // 200/3 ~ 66 Hz -> effectively 60 fps of data
        val ranges = FloatArray(config.scanBins)
        val intensity = ByteArray(config.scanBins)
        val poseBuf = FloatArray(3)
        val odomBuf = FloatArray(3)
        val statusBuf = FloatArray(6)
        val sonarBuf = FloatArray(3)
        val metaBuf = FloatArray(4)
        var lastScanId = -1L
        var tick = 0

        while (running.get()) {
            val stepStartNs = System.nanoTime()
            engine.step(dt)

            if (tick % publishEvery == 0) {
                engine.pose().copyInto(poseBuf)
                engine.odom().copyInto(odomBuf)
                engine.status().copyInto(statusBuf)
                engine.sonar().copyInto(sonarBuf)

                _telemetry.value = Telemetry(
                    pose = Pose(poseBuf[0], poseBuf[1], poseBuf[2]),
                    odomPose = Pose(odomBuf[0], odomBuf[1], odomBuf[2]),
                    gyroZ = statusBuf[0],
                    batteryV = statusBuf[1],
                    rpmL = statusBuf[2],
                    rpmR = statusBuf[3],
                    ticksL = statusBuf[4].toLong(),
                    ticksR = statusBuf[5].toLong(),
                    sonarFrontM = sonarBuf[0],
                    sonarLeftM = sonarBuf[1],
                    sonarRightM = sonarBuf[2],
                    timestampMs = System.currentTimeMillis(),
                )
            }

            engine.scanMeta().copyInto(metaBuf)
            val scanId = metaBuf[0].toLong()
            if (scanId != lastScanId) {
                lastScanId = scanId
                engine.scanRanges(ranges)
                engine.scanIntensity(intensity)
                _scans.tryEmit(
                    Scan(
                        ranges = ranges.copyOf(),
                        intensity = intensity.copyOf(),
                        scanId = scanId,
                        startS = metaBuf[1].toDouble(),
                        endS = metaBuf[2].toDouble(),
                        rotationHz = metaBuf[3],
                    )
                )
            }

            tick++
            // Pace the loop to real time; a slow frame simply catches up
            // without spiralling (dt is fixed, so physics stay stable).
            val elapsedMs = (System.nanoTime() - stepStartNs) / 1_000_000.0
            val sleepMs = (dt * 1000.0 - elapsedMs).toLong()
            if (sleepMs > 0) Thread.sleep(sleepMs)
        }
    }
}
