package com.vacuum.nav.robot

import com.vacuum.nav.robot.protocol.Packets
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import java.io.IOException
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.InetSocketAddress
import java.util.concurrent.atomic.AtomicBoolean

enum class LinkState { DISCOVERING, CLAIMED, ERROR_LINK, CLOSED }

/**
 * RobotLink over UDP to the ESP32 (docs/PROTOCOL.md). Owns three sockets on
 * one thread:
 *
 *  - cmd   -> board:9901  HELO claim, HBC2 at 20 Hz, PNG1 keepalive >= 2 Hz
 *  <-        HBT2/PNG2/BYE1 (HACK replies to the HELO source)
 *  <- telem (local 9902)  HBT2 at 50 Hz
 *  <- scan  (local 9900)  LDS2 one datagram per revolution
 *
 * Safety (prompt §7): no telemetry for [telemetryTimeoutMs] or no lidar for
 * [lidarTimeoutMs] -> zero RPM + ERROR_LINK. The ESP32's own 300 ms watchdog
 * is the backstop if this thread dies entirely.
 */
class UdpRobotLink(
    private val config: RobotConfig,
    private val boardHost: String,
    private val takeover: Boolean = false,
) : RobotLink {

    private val _telemetry = MutableStateFlow(Telemetry())
    override val telemetry: StateFlow<Telemetry> = _telemetry.asStateFlow()

    private val _scans = MutableSharedFlow<Scan>(replay = 1, extraBufferCapacity = 4)
    override val scans: SharedFlow<Scan> = _scans.asSharedFlow()

    private val _state = MutableStateFlow(LinkState.DISCOVERING)
    val state: StateFlow<LinkState> = _state.asStateFlow()

    private val running = AtomicBoolean(true)

    @Volatile private var rpmL = 0f
    @Volatile private var rpmR = 0f
    @Volatile private var estopRequested = false
    @Volatile private var lastTelemMs = 0L
    @Volatile private var lastScanMs = 0L
    @Volatile private var lastHack: Packets.Hack? = null

    val hack: Packets.Hack? get() = lastHack

    private val thread = Thread(::loop, "udp-robot-link").apply {
        isDaemon = true
        start()
    }

    override fun setWheelRpm(rpmL: Float, rpmR: Float) {
        this.rpmL = rpmL
        this.rpmR = rpmR
    }

    fun setEstop(on: Boolean) {
        estopRequested = on
        if (on) setWheelRpm(0f, 0f)
    }

    override fun close() {
        running.set(false)
        thread.join(1000)
        _state.value = LinkState.CLOSED
    }

    private fun loop() {
        val session = (System.nanoTime() and 0xFFFFFFFFL)
        var cmd: DatagramSocket? = null
        var telem: DatagramSocket? = null
        var scan: DatagramSocket? = null
        try {
            val boardAddr = InetAddress.getByName(boardHost)
            cmd = DatagramSocket().apply {
                soTimeout = 50   // keeps the 20 Hz command cadence honest
            }
            // Local ports we ANNOUNCE in HELO; the board sends HBT2 here and
            // LDS2 to the scan port (docs/PROTOCOL.md §3.1).
            telem = DatagramSocket(null).apply {
                reuseAddress = true
                bind(InetSocketAddress(config.boardTelemPort))
                soTimeout = 20
            }
            scan = DatagramSocket(null).apply {
                reuseAddress = true
                bind(InetSocketAddress(config.boardScanPort))
                soTimeout = 20
            }

            val boardEp = InetSocketAddress(boardAddr, config.boardCmdPort)
            val buf = ByteArray(2048)

            var seq = 0
            var pingSeq = 0L
            var lastHbc2 = 0L
            var lastPing = 0L
            var lastHelo = 0L
            var claimed = false

            fun send(bytes: ByteArray, ep: InetSocketAddress) {
                cmd!!.send(DatagramPacket(bytes, bytes.size, ep))
            }

            fun sendHelo() {
                send(Packets.buildHelo(2, config.boardScanPort, config.boardTelemPort,
                    takeover, session), boardEp)
                lastHelo = System.currentTimeMillis()
            }

            fun sendZero() {
                seq++
                send(Packets.buildHbc2(seq, 0, 0,
                    if (estopRequested) 0x0005 else 0x0001, nowMs()), boardEp)
            }

            sendHelo()

            while (running.get()) {
                val now = System.currentTimeMillis()

                // --- HELO retry until HACK granted (1 s cadence) ---
                if (!claimed && now - lastHelo >= 1000) sendHelo()

                // --- PNG1 keepalive >= 2 Hz (lease + clock) ---
                if (now - lastPing >= 500) {
                    lastPing = now
                    pingSeq++
                    send(Packets.buildPing1(pingSeq, nowMs()), boardEp)
                }

                // --- HBC2 at 20 Hz while claimed ---
                if (claimed && now - lastHbc2 >= 1000L / config.commandRateHz) {
                    lastHbc2 = now
                    seq++
                    val flags = if (estopRequested) 0x0005 else 0x0001
                    val (l, r) = if (estopRequested) Pair(0f, 0f) else Pair(rpmL, rpmR)
                    send(Packets.buildHbc2(seq,
                        (l * 10).toInt().coerceIn(-32768, 32767),
                        (r * 10).toInt().coerceIn(-32768, 32767),
                        flags, nowMs()), boardEp)
                }

                // --- safety timeouts (prompt §7): zero + ERROR_LINK ---
                if (claimed) {
                    val telemSilence = now - lastTelemMs
                    val scanSilence = now - lastScanMs
                    val linkDead = lastTelemMs > 0 &&
                        telemSilence > config.telemetryTimeoutMs
                    val lidarDead = lastScanMs > 0 &&
                        scanSilence > config.lidarTimeoutMs
                    if (linkDead || lidarDead) {
                        if (_state.value != LinkState.ERROR_LINK) {
                            _state.value = LinkState.ERROR_LINK
                            sendZero()   // belt and braces; the board also
                                         // stops on its own 300 ms watchdog
                        }
                    } else if (_state.value != LinkState.CLAIMED) {
                        _state.value = LinkState.CLAIMED
                    }
                }

                // --- drain cmd socket: HACK / HBT2 / PNG2 / BYE1 ---
                try {
                    while (true) {
                        val p = DatagramPacket(buf, buf.size)
                        cmd!!.receive(p)
                        val d = buf.copyOf(p.length)
                        when (String(d, 0, 4, Charsets.US_ASCII)) {
                            "HACK" -> {
                                val h = Packets.decodeHack(d) ?: continue
                                lastHack = h
                                if (h.granted && h.session == session) {
                                    claimed = true
                                    lastTelemMs = System.currentTimeMillis()
                                    _state.value = LinkState.CLAIMED
                                }
                            }
                            "HBT2", "HBT1" -> {
                                val t = Packets.decodeTelemetry(d) ?: continue
                                lastTelemMs = System.currentTimeMillis()
                                applyTelemetry(t, lastTelemMs)
                            }
                            "BYE1" -> {
                                claimed = false
                                _state.value = LinkState.DISCOVERING
                                lastHelo = 0   // re-claim immediately
                            }
                            // PNG2: RTT samples are consumed by Diagnostics
                            // (lowest-RTT wins, prompt §3); nothing to do here.
                        }
                    }
                } catch (_: java.net.SocketTimeoutException) {
                } catch (e: IOException) {
                    if (!running.get()) break
                }

                // --- drain telem socket (HBT2 arrives here per HELO) ---
                try {
                    while (true) {
                        val p = DatagramPacket(buf, buf.size)
                        telem!!.receive(p)
                        val d = buf.copyOf(p.length)
                        val t = Packets.decodeTelemetry(d) ?: continue
                        lastTelemMs = System.currentTimeMillis()
                        applyTelemetry(t, lastTelemMs)
                    }
                } catch (_: java.net.SocketTimeoutException) {
                } catch (e: IOException) {
                    if (!running.get()) break
                }

                // --- drain scan socket (LDS2) ---
                try {
                    while (true) {
                        val p = DatagramPacket(buf, buf.size)
                        scan!!.receive(p)
                        val d = buf.copyOf(p.length)
                        val s = Packets.decodeScan(d) ?: continue
                        lastScanMs = System.currentTimeMillis()
                        _scans.tryEmit(Scan(
                            ranges = s.ranges,
                            intensity = s.intensity,
                            scanId = s.scanId,
                            startS = s.startMs / 1000.0,
                            endS = s.endMs / 1000.0,
                            rotationHz = s.rotationHz,
                        ))
                    }
                } catch (_: java.net.SocketTimeoutException) {
                } catch (e: IOException) {
                    if (!running.get()) break
                }
            }
            // Graceful close: stop the wheels, then let the lease expire.
            if (claimed) {
                seq++
                send(Packets.buildHbc2(seq, 0, 0, 0x0001, nowMs()), boardEp)
            }
        } catch (e: Exception) {
            // Bad host / port bind: surface as ERROR_LINK, never crash the
            // service (prompt §7: the app must fail safe).
            _state.value = LinkState.ERROR_LINK
        } finally {
            runCatching { cmd?.close() }
            runCatching { telem?.close() }
            runCatching { scan?.close() }
        }
    }

    private fun nowMs(): Long = System.currentTimeMillis() and 0xFFFFFFFFL

    /**
     * Copy wire scalars into the shared Telemetry. Pose/odomPose stay put:
     * they are owned by the odometry + EKF module (Phase 2 engine), which
     * consumes ticks/gyro from here at its own rate.
     */
    private fun applyTelemetry(t: Packets.Telemetry, atMs: Long) {
        _telemetry.value = _telemetry.value.copy(
            gyroZ = t.gyroMilliDps[2] / 16000f,  // raw = deg/s * 16 -> rad/s
            batteryV = t.batteryV.toFloat(),
            rpmL = t.rpmL.toFloat(),
            rpmR = t.rpmR.toFloat(),
            ticksL = t.ticksL.toLong(),
            ticksR = t.ticksR.toLong(),
            sonarFrontM = t.sonarFrontMm / 1000f,   // wire mm -> SI metres
            sonarLeftM = t.sonarLeftMm / 1000f,
            sonarRightM = t.sonarRightMm / 1000f,
            timestampMs = atMs,
        )
    }
}
