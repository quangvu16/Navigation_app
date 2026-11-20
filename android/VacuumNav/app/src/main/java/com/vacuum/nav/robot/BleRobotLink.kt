package com.vacuum.nav.robot

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import com.vacuum.nav.robot.protocol.Packets
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import java.util.UUID
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicBoolean

/**
 * RobotLink over Bluetooth LE, using the FriendBot GATT layout (UUIDs
 * verbatim from FriendBot ESP32S3 firmware): UART-style service, write-
 * without-response RX (phone -> board), notify TX (board -> phone).
 *
 * Wire framing over TX (firmware ble_link.h):
 *   [0x00][datagram]        one whole packet (HBT2/HACK/PNG2/BYE1)
 *   [0x01][idx][count][..]  LDS2 split for the BLE MTU
 * RX writes are one datagram each (GATT write = boundary).
 *
 * Same safety contract as UdpRobotLink: HELO claim, PNG1 keepalive >= 2 Hz,
 * HBC2 at 20 Hz, telemetry/lidar timeouts -> zero RPM + ERROR_LINK.
 */
@SuppressLint("MissingPermission")
class BleRobotLink(
    private val context: Context,
    private val config: RobotConfig,
    private val takeover: Boolean = false,
) : RobotLink {
    override val description: String get() = "BLE · " + deviceAddress.ifEmpty { "connecting…" }


    companion object {
        // FriendBot's UUIDs (firmware/ble_link.h).
        val SERVICE_UUID: UUID = UUID.fromString("61653dc3-4021-4d1e-ba83-8b4eec61d613")
        val RX_UUID: UUID = UUID.fromString("06386c14-86ea-4d71-811c-48f97c58f8c9")
        val TX_UUID: UUID = UUID.fromString("9bf1103b-834c-47cf-b149-c9e4bcf778a7")
        private val CCCD_UUID: UUID =
            UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

        private const val FRAME_SINGLE = 0x00
        private const val FRAME_CHUNK = 0x01

        /** Scans for boards advertising the FriendBot service. */
        fun scan(context: Context, timeoutMs: Long = 5000L,
                 onResult: (List<ScanResult>) -> Unit) {
            val mgr = context.getSystemService(BluetoothManager::class.java)
            val scanner = mgr?.adapter?.bluetoothLeScanner ?: run {
                onResult(emptyList()); return
            }
            val found = LinkedHashMap<String, ScanResult>()
            val cb = object : ScanCallback() {
                override fun onScanResult(callbackType: Int, result: ScanResult) {
                    if (result.scanRecord?.serviceUuids?.any {
                            it.uuid == SERVICE_UUID
                        } == true
                    ) {
                        found[result.device.address] = result
                        onResult(found.values.toList())
                    }
                }
                override fun onScanFailed(errorCode: Int) {
                    onResult(found.values.toList())
                }
            }
            scanner.startScan(
                null,
                ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build(),
                cb,
            )
            android.os.Handler(android.os.Looper.getMainLooper()).postDelayed({
                runCatching { scanner.stopScan(cb) }
                onResult(found.values.toList())
            }, timeoutMs)
        }
    }

    private val _telemetry = MutableStateFlow(Telemetry())
    override val telemetry: StateFlow<Telemetry> = _telemetry.asStateFlow()

    private val _scans = MutableSharedFlow<Scan>(replay = 1, extraBufferCapacity = 4)
    override val scans: SharedFlow<Scan> = _scans.asSharedFlow()

    private val _state = MutableStateFlow(LinkState.DISCOVERING)
    override val state: StateFlow<LinkState> = _state.asStateFlow()

    private val running = AtomicBoolean(true)
    private val txQueue = ConcurrentLinkedQueue<ByteArray>()

    @Volatile private var gatt: BluetoothGatt? = null
    @Volatile private var rxChar: BluetoothGattCharacteristic? = null
    @Volatile private var rpmL = 0f
    @Volatile private var rpmR = 0f
    @Volatile private var estopRequested = false
    @Volatile private var buzzer = false
    @Volatile private var lastTelemMs = 0L
    @Volatile private var lastScanMs = 0L
    @Volatile private var claimed = false

    // LDS2 chunk reassembly.
    private val scanChunks = ArrayList<ByteArray?>(8)
    private var scanChunkCount = 0

    private val session = (System.nanoTime() and 0xFFFFFFFFL)

    private val thread = Thread(::loop, "ble-robot-link").apply {
        isDaemon = true
        start()
    }

    /** Start connecting to the picked device (set after construction so the
     *  Connect dialog can supply the address without a race). */
    fun start(address: String) {
        deviceAddress = address
        connect()
    }

    override fun setWheelRpm(rpmL: Float, rpmR: Float) {
        this.rpmL = rpmL
        this.rpmR = rpmR
    }

    fun setEstop(on: Boolean) {
        estopRequested = on
        if (on) setWheelRpm(0f, 0f)
    }

    override fun setBuzzer(on: Boolean) {
        buzzer = on
    }

    override fun close() {
        running.set(false)
        thread.join(1000)
        runCatching { gatt?.disconnect() }
        runCatching { gatt?.close() }
        _state.value = LinkState.CLOSED
    }

    // ------------------------------------------------------------- GATT
    private val gattCb = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                g.discoverServices()
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                claimed = false
                _state.value = LinkState.DISCOVERING
                if (running.get()) {
                    runCatching { g.close() }
                    gatt = null
                    android.os.Handler(android.os.Looper.getMainLooper())
                        .postDelayed({ if (running.get()) connect() }, 1000)
                }
            }
        }

        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) return
            val svc = g.getService(SERVICE_UUID) ?: return
            val tx = svc.getCharacteristic(TX_UUID) ?: return
            rxChar = svc.getCharacteristic(RX_UUID)
            g.setCharacteristicNotification(tx, true)
            val cccd = tx.getDescriptor(CCCD_UUID)
            if (cccd != null) {
                cccd.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                g.writeDescriptor(cccd)
            }
            send(Packets.buildHelo(2, config.boardScanPort, config.boardTelemPort,
                takeover, session))   // claim over BLE (same datagram as UDP)
        }

        @Deprecated("Deprecated in Java")
        override fun onCharacteristicChanged(
            g: BluetoothGatt, characteristic: BluetoothGattCharacteristic,
        ) {
            onFrame(characteristic.value ?: return)
        }

        override fun onCharacteristicChanged(
            g: BluetoothGatt, characteristic: BluetoothGattCharacteristic,
            value: ByteArray,
        ) {
            onFrame(value)
        }
    }

    private fun connect() {
        if (!running.get()) return
        val mgr = context.getSystemService(BluetoothManager::class.java)
        val adapter = mgr?.adapter
        if (adapter == null || !adapter.isEnabled) {
            _state.value = LinkState.ERROR_LINK
            return
        }
        runCatching {
            val dev: BluetoothDevice = adapter.getRemoteDevice(deviceAddress)
            gatt = dev.connectGatt(context, false, gattCb, BluetoothDevice.TRANSPORT_LE)
        }.onFailure { _state.value = LinkState.ERROR_LINK }
    }

    /** Set by the Connect dialog (the picked scan result's address). */
    @Volatile
    var deviceAddress: String = ""

    private fun send(bytes: ByteArray) {
        txQueue.add(bytes)
    }

    // ------------------------------------------------------------ frames
    private fun onFrame(value: ByteArray) {
        if (value.isEmpty()) return
        when (value[0].toInt() and 0xFF) {
            FRAME_SINGLE -> handleDatagram(value.copyOfRange(1, value.size))
            FRAME_CHUNK -> {
                if (value.size < 3) return
                val idx = value[1].toInt() and 0xFF
                val count = value[2].toInt() and 0xFF
                if (count !in 1..8) return
                if (scanChunks.size != count) {
                    scanChunks.clear()
                    repeat(count) { scanChunks.add(null) }
                    scanChunkCount = count
                }
                scanChunks[idx] = value.copyOfRange(3, value.size)
                if (scanChunks.all { it != null }) {
                    val joined = scanChunks.filterNotNull()
                        .fold(ByteArray(0)) { acc, b -> acc + b }
                    scanChunks.clear()
                    scanChunkCount = 0
                    handleDatagram(joined)
                }
            }
        }
    }

    private fun handleDatagram(d: ByteArray) {
        when (val magic = String(d, 0, minOf(4, d.size), Charsets.US_ASCII)) {
            "HBT2", "HBT1" -> {
                val t = Packets.decodeTelemetry(d) ?: return
                lastTelemMs = System.currentTimeMillis()
                _telemetry.value = _telemetry.value.copy(
                    gyroZ = t.gyroMilliDps[2] / 16000f,
                    batteryV = t.batteryV.toFloat(),
                    rpmL = t.rpmL.toFloat(),
                    rpmR = t.rpmR.toFloat(),
                    ticksL = t.ticksL.toLong(),
                    ticksR = t.ticksR.toLong(),
                    sonarFrontM = t.sonarFrontMm / 1000f,
                    sonarLeftM = t.sonarLeftMm / 1000f,
                    sonarRightM = t.sonarRightMm / 1000f,
                    timestampMs = lastTelemMs,
                )
                if (!claimed && t.isV2) claimed = true
                if (_state.value != LinkState.CLAIMED && claimed)
                    _state.value = LinkState.CLAIMED
            }
            "HACK" -> {
                val h = Packets.decodeHack(d) ?: return
                if (h.granted && h.session == session) {
                    claimed = true
                    lastTelemMs = System.currentTimeMillis()
                    _state.value = LinkState.CLAIMED
                }
            }
            "LDS2", "LDS1" -> {
                val s = Packets.decodeScan(d) ?: return
                lastScanMs = System.currentTimeMillis()
                _scans.tryEmit(Scan(
                    ranges = s.ranges, intensity = s.intensity,
                    scanId = s.scanId, startS = s.startMs / 1000.0,
                    endS = s.endMs / 1000.0, rotationHz = s.rotationHz,
                ))
            }
            "BYE1" -> {
                claimed = false
                _state.value = LinkState.DISCOVERING
            }
            else -> Unit   // PNG2 etc: consumed by Diagnostics later
        }
    }

    // -------------------------------------------------------------- loop
    private fun loop() {
        var seq = 0
        var pingSeq = 0L
        var lastHbc2 = 0L
        var lastPing = 0L
        var lastHelo = 0L
        val phoneT0 = System.currentTimeMillis()

        while (running.get()) {
            val now = System.currentTimeMillis()

            if (!claimed && now - lastHelo >= 1000) {
                lastHelo = now
                send(Packets.buildHelo(2, config.boardScanPort,
                    config.boardTelemPort, takeover, session))
            }
            if (now - lastPing >= 500) {
                lastPing = now
                pingSeq++
                send(Packets.buildPing1(pingSeq, now - phoneT0))
            }
            if (claimed && now - lastHbc2 >= 1000L / config.commandRateHz) {
                lastHbc2 = now
                seq++
                var flags = if (estopRequested) 0x0005 else 0x0001
                if (buzzer) flags = flags or 0x0002
                val (l, r) = if (estopRequested) Pair(0f, 0f) else Pair(rpmL, rpmR)
                send(Packets.buildHbc2(seq,
                    (l * 10).toInt().coerceIn(-32768, 32767),
                    (r * 10).toInt().coerceIn(-32768, 32767),
                    flags, now - phoneT0))
            }

            // Safety timeouts (prompt §7).
            if (claimed) {
                val telemDead = lastTelemMs > 0 &&
                    now - lastTelemMs > config.telemetryTimeoutMs
                val lidarDead = lastScanMs > 0 &&
                    now - lastScanMs > config.lidarTimeoutMs
                if (telemDead || lidarDead) {
                    if (_state.value != LinkState.ERROR_LINK) {
                        _state.value = LinkState.ERROR_LINK
                        send(Packets.buildHbc2(++seq, 0, 0,
                            if (estopRequested) 0x0005 else 0x0001,
                            now - phoneT0))
                    }
                } else if (_state.value != LinkState.CLAIMED) {
                    _state.value = LinkState.CLAIMED
                }
            }

            // Drain TX queue over GATT (write-without-response).
            val rx = rxChar
            val g = gatt
            if (rx != null && g != null) {
                while (true) {
                    val pkt = txQueue.poll() ?: break
                    runCatching {
                        rx.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
                        rx.value = pkt
                        g.writeCharacteristic(rx)
                    }
                }
            }

            try {
                Thread.sleep(20)
            } catch (_: InterruptedException) {
                break
            }
        }
    }
}
