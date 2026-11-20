package com.vacuum.nav.robot.protocol

/**
 * Kotlin decoders for the phone<->ESP32 wire protocol (docs/PROTOCOL.md).
 * Exact counterparts of firmware/vacuum_phone_esp32/phone_link.h and
 * lidar_ld14.h; unit tests are built from captured hex (see test resources).
 *
 * Rules (prompt §4 / PROTOCOL.md §4): unknown magic or short datagram ->
 * null (drop + count in the link), never throw. Little-endian throughout.
 */
object Packets {

    // ---- HBT2 (96 B) / HBT1 (78 B, accepted for the transition period) ----

    data class Telemetry(
        val isV2: Boolean,
        val seq: Int,
        val boardMs: Long,
        val flags: Int,
        val fbBytes: Int,
        val ticksL: Int,
        val ticksR: Int,
        val rpmL: Int,
        val rpmR: Int,
        val batteryCentiV: Int,
        val boardTempDeciC: Int,
        val currLeftCentiA: Int,
        val currRightCentiA: Int,
        val frameCount: Long,
        val checksumErr: Long,
        val sonarFrontMm: Int,
        val sonarLeftMm: Int,
        val sonarRightMm: Int,
        val imuStatus: Int,
        val quat: IntArray,      // w,x,y,z scale 1/16384
        val gyroMilliDps: IntArray,  // x,y,z: raw/16 deg/s -> keep raw as centi
        val accelCentiMs2: IntArray, // x,y,z scale 1/100 m/s^2
        val batState: Int,
        val batCells: Int,
        val batFilteredCentiV: Int,
        // v2 extension (0 when HBT1)
        val lastCmdSeq: Int = 0,
        val lastCmdPhoneMs: Long = 0,
        val cmdAgeMs: Int = 0xFFFF,
        val linkFlags: Int = 0,
        val rssiDbm: Int = 0,
        val sonarReflex: Int = 0,
        val lidarDeciHz: Int = 0,
        val outRpmLx10: Int = 0,
        val outRpmRx10: Int = 0,
        val lidarCrcErr: Int = 0,
    ) {
        val estopLatched get() = linkFlags and 0x01 != 0
        val webOwns get() = linkFlags and 0x02 != 0
        val otaActive get() = linkFlags and 0x04 != 0
        val apMode get() = linkFlags and 0x08 != 0
        val staConnected get() = linkFlags and 0x10 != 0
        val mainboardUp get() = flags and 0x0001 != 0
        val cmdTimeout get() = flags and 0x0002 != 0
        val batteryV get() = batteryCentiV / 100.0
        val lidarHz get() = lidarDeciHz / 10.0
    }

    private fun le16(b: ByteArray, off: Int): Int =
        (b[off].toInt() and 0xFF) or ((b[off + 1].toInt() and 0xFF) shl 8)

    private fun le16s(b: ByteArray, off: Int): Int {
        val v = le16(b, off)
        return if (v >= 0x8000) v - 0x10000 else v
    }

    private fun le32(b: ByteArray, off: Int): Long =
        (b[off].toLong() and 0xFF) or
            ((b[off + 1].toLong() and 0xFF) shl 8) or
            ((b[off + 2].toLong() and 0xFF) shl 16) or
            ((b[off + 3].toLong() and 0xFF) shl 24)

    /**
     * Decodes HBT2 (96 B) or HBT1 (78/74/52/44/42 B historical lengths —
     * the ROS bridge accepted all five, and so do we). Returns null on
     * anything else.
     */
    fun decodeTelemetry(b: ByteArray): Telemetry? {
        if (b.size < 42) return null
        val magic = String(b, 0, 4, Charsets.US_ASCII)
        val isV2 = when (magic) {
            // HBT2 is fixed at 96 B; a shorter one is corrupt, not legacy.
            "HBT2" -> if (b.size >= 96) true else return null
            "HBT1" -> false
            else -> return null
        }
        val seq = le16(b, 4)
        val boardMs = le32(b, 6)
        val flags = le16(b, 10)
        val fbBytes = le16(b, 12)
        val ticksL = le32(b, 14).toInt()
        val ticksR = le32(b, 18).toInt()
        val rpmL = le16s(b, 22)
        val rpmR = le16s(b, 24)
        val batt = if (b.size >= 28) le16s(b, 26) else 0
        val temp = if (b.size >= 30) le16s(b, 28) else 0
        val curL = if (b.size >= 32) le16s(b, 30) else 0
        val curR = if (b.size >= 34) le16s(b, 32) else 0
        val frames = if (b.size >= 38) le32(b, 34) else 0
        val csum = if (b.size >= 42) le32(b, 38) else 0
        val sonarF = if (b.size >= 44) le16(b, 42) else 0
        val sonarL = if (b.size >= 50) le16(b, 48) else 0
        val sonarR = if (b.size >= 52) le16(b, 50) else 0
        val imuStatus = if (b.size >= 54) le16(b, 52) else 0
        val quat = IntArray(4) { if (b.size >= 62) le16s(b, 54 + it * 2) else 0 }
        val gyro = IntArray(3) { if (b.size >= 68) le16s(b, 62 + it * 2) else 0 }
        val accel = IntArray(3) { if (b.size >= 74) le16s(b, 68 + it * 2) else 0 }
        val batState = if (b.size >= 75) b[74].toInt() and 0xFF else 0
        val batCells = if (b.size >= 76) b[75].toInt() and 0xFF else 0
        val batFilt = if (b.size >= 78) le16(b, 76) else 0

        return Telemetry(
            isV2 = isV2,
            seq = seq,
            boardMs = boardMs,
            flags = flags,
            fbBytes = fbBytes,
            ticksL = ticksL,
            ticksR = ticksR,
            rpmL = rpmL,
            rpmR = rpmR,
            batteryCentiV = batt,
            boardTempDeciC = temp,
            currLeftCentiA = curL,
            currRightCentiA = curR,
            frameCount = frames,
            checksumErr = csum,
            sonarFrontMm = sonarF,
            sonarLeftMm = sonarL,
            sonarRightMm = sonarR,
            imuStatus = imuStatus,
            quat = quat,
            gyroMilliDps = gyro,
            accelCentiMs2 = accel,
            batState = batState,
            batCells = batCells,
            batFilteredCentiV = batFilt,
            lastCmdSeq = if (isV2) le16(b, 78) else 0,
            lastCmdPhoneMs = if (isV2) le32(b, 80) else 0,
            cmdAgeMs = if (isV2) le16(b, 84) else 0xFFFF,
            linkFlags = if (isV2) b[86].toInt() and 0xFF else 0,
            rssiDbm = if (isV2) b[87].toInt() else 0,
            sonarReflex = if (isV2) b[88].toInt() and 0xFF else 0,
            lidarDeciHz = if (isV2) b[89].toInt() and 0xFF else 0,
            outRpmLx10 = if (isV2) le16s(b, 90) else 0,
            outRpmRx10 = if (isV2) le16s(b, 92) else 0,
            lidarCrcErr = if (isV2) le16(b, 94) else 0,
        )
    }

    // ---- LDS2 (26 + 3*bins) / LDS1 (14 + 2*bins, accepted) ----

    data class Scan(
        val ranges: FloatArray,     // metres, 0f = no return (wire convention)
        val intensity: ByteArray,
        val scanId: Long,
        val startMs: Long,
        val endMs: Long,
        val rotationHz: Float,
        val points: Int,
        val valid: Int,
        val isV2: Boolean,
    )

    fun decodeScan(b: ByteArray): Scan? {
        if (b.size < 14) return null
        val magic = String(b, 0, 4, Charsets.US_ASCII)
        val isV2 = magic == "LDS2"
        if (!isV2 && magic != "LDS1") return null
        val bins = le16(b, 4)
        if (bins == 0 || bins > 720) return null
        if (isV2) {
            if (b.size < 26 + bins * 2) return null
            val scanId = le32(b, 6)
            val startMs = le32(b, 10)
            val endMs = le32(b, 14)
            val centiHz = le16(b, 18)
            val points = le16(b, 20)
            val valid = le16(b, 22)
            val flags = le16(b, 24)
            val ranges = FloatArray(bins) {
                val mm = le16(b, 26 + it * 2)
                if (mm == 0) 0f else mm / 1000f
            }
            val intensity = if (flags and 1 != 0 && b.size >= 26 + bins * 3) {
                ByteArray(bins) { b[26 + bins * 2 + it] }
            } else {
                ByteArray(bins)
            }
            return Scan(ranges, intensity, scanId, startMs, endMs,
                centiHz / 100f, points, valid, true)
        }
        // LDS1: reserved bytes 10..13, no timestamps, no intensity.
        if (b.size < 14 + bins * 2) return null
        val scanId = le32(b, 6)
        val ranges = FloatArray(bins) {
            val mm = le16(b, 14 + it * 2)
            if (mm == 0) 0f else mm / 1000f
        }
        return Scan(ranges, ByteArray(bins), scanId, 0, 0, 0f, 0, 0, false)
    }

    // ---- HACK / PNG2 / BYE1 ----

    data class Hack(
        val proto: Int,
        val status: Int,       // 0 granted, 1 denied, 2 version mismatch
        val boardType: Int,    // 0 WROOM, 1 S3
        val scanBins: Int,
        val ticksPerRev: Int,
        val session: Long,
        val boardMs: Long,
        val fwVersion: String,
    ) {
        val granted get() = status == 0
    }

    fun decodeHack(b: ByteArray): Hack? {
        if (b.size < 21 || String(b, 0, 4, Charsets.US_ASCII) != "HACK") return null
        val nul = (20 until minOf(b.size, 52)).firstOrNull { b[it] == 0.toByte() }
            ?: 52
        return Hack(
            proto = le16(b, 4),
            status = b[6].toInt() and 0xFF,
            boardType = b[7].toInt() and 0xFF,
            scanBins = le16(b, 8),
            ticksPerRev = le16(b, 10),
            session = le32(b, 12),
            boardMs = le32(b, 16),
            fwVersion = String(b, 20, nul - 20, Charsets.US_ASCII),
        )
    }

    data class Ping2(val seq: Long, val phoneMs: Long, val espMs: Long, val espUs: Long)

    fun decodePing2(b: ByteArray): Ping2? {
        if (b.size < 20 || String(b, 0, 4, Charsets.US_ASCII) != "PNG2") return null
        return Ping2(le32(b, 4), le32(b, 8), le32(b, 12), le32(b, 16))
    }

    data class Bye(val reason: Int)

    fun decodeBye(b: ByteArray): Bye? {
        if (b.size < 5 || String(b, 0, 4, Charsets.US_ASCII) != "BYE1") return null
        return Bye(b[4].toInt() and 0xFF)
    }

    // ---- Wi-Fi provisioning / OTA replies (docs/PROTOCOL.md §3.9/§3.10) ----

    /** WFCA status (WFC1 reply): 0 ok, 1 bad length, 2 not owner, 4 AP-only. */
    data class Wfca(val status: Int) {
        val ok get() = status == 0
    }

    fun decodeWfca(b: ByteArray): Wfca? {
        if (b.size < 5 || String(b, 0, 4, Charsets.US_ASCII) != "WFCA") return null
        return Wfca(b[4].toInt() and 0xFF)
    }

    /** WFOA status: 0 accepted, 1 not owner, 2 bad password, 3 moving,
     *  4 bad url, 5 ota disabled, 7 busy. */
    data class Wfoa(val status: Int) {
        val accepted get() = status == 0
    }

    fun decodeWfoa(b: ByteArray): Wfoa? {
        if (b.size < 5 || String(b, 0, 4, Charsets.US_ASCII) != "WFOA") return null
        return Wfoa(b[4].toInt() and 0xFF)
    }

    /** WFOB final result: 0 flashed+rebooting, 1 download/write failed,
     *  2 no update at URL; detail = HTTPUpdate error code. */
    data class Wfob(val status: Int, val detail: Int) {
        val ok get() = status == 0
    }

    fun decodeWfob(b: ByteArray): Wfob? {
        if (b.size < 6 || String(b, 0, 4, Charsets.US_ASCII) != "WFOB") return null
        return Wfob(b[4].toInt() and 0xFF, b[5].toInt() and 0xFF)
    }

    // ---- builders (phone -> board) ----

    /** WFC1: stage SSID+password on the board (clear = empty ssid). */
    fun buildWfc1(ssid: String, pass: String, clear: Boolean = false): ByteArray {
        val s = if (clear) ByteArray(0) else ssid.toByteArray(Charsets.UTF_8)
            .also { require(it.size <= 32) { "ssid too long" } }
        val p = if (clear) ByteArray(0) else pass.toByteArray(Charsets.UTF_8)
            .also { require(it.size <= 64) { "password too long" } }
        return ByteBufferLE.build(7 + s.size + p.size) {
            ascii("WFC1")
            u8(if (clear) 0x01 else 0)
            u8(s.size)
            u8(p.size)
            write(s)
            write(p)
        }
    }

    /** WFO1: pull-update the board from url (password = OTA_PASSWORD). */
    fun buildWfo1(url: String, otaPass: String): ByteArray {
        val u = url.toByteArray(Charsets.US_ASCII)
            .also { require(it.size in 8..200) { "bad url length" } }
        val p = otaPass.toByteArray(Charsets.UTF_8)
            .also { require(it.size <= 64) { "password too long" } }
        return ByteBufferLE.build(6 + u.size + p.size) {
            ascii("WFO1")
            u8(u.size)
            u8(p.size)
            write(u)
            write(p)
        }
    }

    fun buildHelo(proto: Int, scanPort: Int, telemPort: Int, takeover: Boolean,
                  session: Long): ByteArray =
        ByteBufferLE.build(16) {
            ascii("HELO")
            u16(proto)
            u16(scanPort)
            u16(telemPort)
            u16(if (takeover) 0x0001 else 0)
            u32(session)
        }

    fun buildHbc2(seq: Int, rpmLx10: Int, rpmRx10: Int, flags: Int,
                  phoneMs: Long): ByteArray =
        ByteBufferLE.build(16) {
            ascii("HBC2")
            u16(seq)
            i16(rpmLx10)
            i16(rpmRx10)
            u16(flags)
            u32(phoneMs)
        }

    fun buildPing1(seq: Long, phoneMs: Long): ByteArray =
        ByteBufferLE.build(12) {
            ascii("PNG1")
            u32(seq)
            u32(phoneMs)
        }

    /** Tiny little-endian builder so the builders above stay readable. */
    private object ByteBufferLE {
        fun build(size: Int, fill: java.io.ByteArrayOutputStream.() -> Unit): ByteArray {
            val out = java.io.ByteArrayOutputStream(size)
            out.fill()
            return out.toByteArray()
        }
    }

    private fun java.io.ByteArrayOutputStream.ascii(s: String) =
        write(s.toByteArray(Charsets.US_ASCII))

    private fun java.io.ByteArrayOutputStream.u8(v: Int) =
        write(v and 0xFF)

    private fun java.io.ByteArrayOutputStream.u16(v: Int) {
        write(v and 0xFF)
        write((v shr 8) and 0xFF)
    }

    private fun java.io.ByteArrayOutputStream.i16(v: Int) = u16(v)

    private fun java.io.ByteArrayOutputStream.u32(v: Long) {
        write((v and 0xFF).toInt())
        write(((v shr 8) and 0xFF).toInt())
        write(((v shr 16) and 0xFF).toInt())
        write(((v shr 24) and 0xFF).toInt())
    }
}
