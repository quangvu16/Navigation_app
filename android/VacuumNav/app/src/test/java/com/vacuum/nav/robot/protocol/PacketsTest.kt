package com.vacuum.nav.robot.protocol

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Decoder tests against byte layouts captured from the firmware source
 * (docs/PROTOCOL.md, written from phone_link.h / lidar_ld14.h). Real
 * over-the-air captures from tools/phone_sim.py --capture replace these
 * vectors once hardware is on the bench; the offsets must not drift.
 */
class PacketsTest {

    // ---- HBT2 -------------------------------------------------------------

    private fun hbt2Fixture(): ByteArray = ByteArray(96).also { b ->
        b.putAscii(0, "HBT2")
        b.putU16(4, 1234)          // telemetry seq
        b.putU32(6, 987654L)       // board ms
        b.putU16(10, 0x0001)       // flags: mainboard up
        b.putU16(12, 26)           // fb_bytes (26-byte FOC frame)
        b.putU32(14, 4242L)        // ticks left
        b.putU32(18, 4241L)        // ticks right
        b.putU16(22, 65535 - 29)   // rpm left = -30 (i16)
        b.putU16(24, 30)           // rpm right = +30
        b.putU16(26, 3900)         // battery 39.00 V
        b.putU16(28, 253)          // 25.3 C
        b.putU16(30, 100)          // curr L 1.00 A
        b.putU16(32, 110)          // curr R 1.10 A
        b.putU32(34, 5000L)        // frames
        b.putU32(38, 2L)           // checksum errors
        b.putU16(42, 350)          // front sonar 350 mm
        b.putU16(48, 800)          // left sonar
        b.putU16(50, 0)            // right sonar: no echo
        b.putU16(52, 0x0003)       // imu: found + fresh
        // quat w,x,y,z = 16384,0,0,0 (identity)
        b.putU16(54, 16384); b.putU16(56, 0); b.putU16(58, 0); b.putU16(60, 0)
        b.putU16(62, 160); b.putU16(64, 0); b.putU16(66, 0)  // gyro: 10.0 deg/s x
        b.putU16(68, 0); b.putU16(70, 0); b.putU16(72, 981)  // accel z ~9.81
        b[74] = 1                  // bat ok
        b[75] = 10                 // cells
        b.putU16(76, 3900)         // filtered
        // extension
        b.putU16(78, 77)           // last cmd seq
        b.putU32(80, 111111L)      // last cmd phone_ms
        b.putU16(84, 50)           // cmd age
        b[86] = 0x01               // link flags: E-STOP latched
        b[87] = 0xC5.toByte()      // rssi -59
        b[88] = 1                  // reflex: stop
        b[89] = 60                 // 6.0 Hz
        b.putU16(90, 65535 - 299)  // out L = -30.0 rpm x10
        b.putU16(92, 300)          // out R = +30.0 rpm x10
        b.putU16(94, 3)            // lidar crc errors
    }

    @Test
    fun hbt2DecodesEveryField() {
        val t = Packets.decodeTelemetry(hbt2Fixture())!!
        assertTrue(t.isV2)
        assertEquals(1234, t.seq)
        assertEquals(987654L, t.boardMs)
        assertTrue(t.mainboardUp)
        assertFalse(t.cmdTimeout)
        assertEquals(26, t.fbBytes)
        assertEquals(4242, t.ticksL)
        assertEquals(4241, t.ticksR)
        assertEquals(-30, t.rpmL)
        assertEquals(30, t.rpmR)
        assertEquals(39.0, t.batteryV, 1e-9)
        assertEquals(350, t.sonarFrontMm)
        assertEquals(800, t.sonarLeftMm)
        assertEquals(0, t.sonarRightMm)
        assertEquals(16384, t.quat[0])
        assertEquals(10 * 16, t.gyroMilliDps[0])   // raw 160 = 10.0 deg/s
        assertEquals(981, t.accelCentiMs2[2])
        assertEquals(1, t.batState)
        assertEquals(10, t.batCells)
        assertEquals(77, t.lastCmdSeq)
        assertEquals(111111L, t.lastCmdPhoneMs)
        assertEquals(50, t.cmdAgeMs)
        assertTrue(t.estopLatched)
        assertFalse(t.webOwns)
        assertEquals(-59, t.rssiDbm)
        assertEquals(1, t.sonarReflex)
        assertEquals(6.0, t.lidarHz, 1e-9)
        assertEquals(-300, t.outRpmLx10)
        assertEquals(300, t.outRpmRx10)
        assertEquals(3, t.lidarCrcErr)
    }

    @Test
    fun hbt1Legacy78StillDecodes() {
        val b = hbt2Fixture().copyOf(78)
        b.putAscii(0, "HBT1")
        val t = Packets.decodeTelemetry(b)!!
        assertFalse(t.isV2)
        assertEquals(4242, t.ticksL)
        assertEquals(39.0, t.batteryV, 1e-9)
        assertEquals(0, t.lastCmdSeq)      // extension absent -> defaults
        assertEquals(0xFFFF, t.cmdAgeMs)
        assertEquals(0, t.linkFlags)
    }

    @Test
    fun hbt1Oldest42ByteVariantDecodes() {
        // The oldest firmware: no sonars, no IMU, no battery state.
        val b = ByteArray(42).also {
            it.putAscii(0, "HBT1")
            it.putU16(4, 1)
            it.putU32(6, 100L)
            it.putU16(10, 0x0001)
            it.putU16(12, 18)
            it.putU32(14, 10L)
            it.putU32(18, 11L)
            it.putU16(22, 5)
            it.putU16(24, 6)
            it.putU16(26, 3700)
            it.putU16(28, 300)
            it.putU16(30, 0)
            it.putU16(32, 0)
            it.putU32(34, 9L)
            it.putU32(38, 0L)
        }
        val t = Packets.decodeTelemetry(b)!!
        assertFalse(t.isV2)
        assertEquals(10, t.ticksL)
        assertEquals(37.0, t.batteryV, 1e-9)
        assertEquals(0, t.sonarFrontMm)     // field absent -> 0
        assertEquals(0, t.batState)
    }

    @Test
    fun telemetryRejectsGarbage() {
        assertNull(Packets.decodeTelemetry(ByteArray(10)))
        assertNull(Packets.decodeTelemetry(ByteArray(96) { 0x42 }))
        assertNull(Packets.decodeTelemetry("XXXX".toByteArray()))
    }

    // ---- LDS2 / LDS1 ------------------------------------------------------

    @Test
    fun lds2DecodesWithIntensity() {
        val bins = 360
        val b = ByteArray(26 + bins * 3)
        b.putAscii(0, "LDS2")
        b.putU16(4, bins)
        b.putU32(6, 42L)          // scan id
        b.putU32(10, 1000L)       // start
        b.putU32(14, 1166L)       // end
        b.putU16(18, 600)         // 6.00 Hz
        b.putU16(20, 1200)        // points
        b.putU16(22, 355)         // valid
        b.putU16(24, 0x0001)      // intensity present
        for (i in 0 until bins) {
            val mm = if (i % 2 == 0) 1500 else 0   // alternate: hit / no return
            b.putU16(26 + i * 2, mm)
            b[26 + bins * 2 + i] = if (i % 2 == 0) 200.toByte() else 0
        }
        val s = Packets.decodeScan(b)!!
        assertTrue(s.isV2)
        assertEquals(42L, s.scanId)
        assertEquals(1000L, s.startMs)
        assertEquals(1166L, s.endMs)
        assertEquals(6.0f, s.rotationHz, 1e-6f)
        assertEquals(1200, s.points)
        assertEquals(355, s.valid)
        assertEquals(1.5f, s.ranges[0], 1e-6f)
        assertEquals(0f, s.ranges[1], 0f)          // no return -> 0
        assertEquals(200.toByte(), s.intensity[0])
        assertEquals(bins, s.ranges.size)
    }

    @Test
    fun lds1LegacyStillDecodes() {
        val bins = 360
        val b = ByteArray(14 + bins * 2)
        b.putAscii(0, "LDS1")
        b.putU16(4, bins)
        b.putU32(6, 7L)
        // bytes 10..13 reserved zero
        b.putU16(14, 2000)
        val s = Packets.decodeScan(b)!!
        assertFalse(s.isV2)
        assertEquals(7L, s.scanId)
        assertEquals(2.0f, s.ranges[0], 1e-6f)
        assertEquals(0f, s.ranges[5], 0f)
    }

    @Test
    fun scanRejectsBadMagicAndShortPackets() {
        assertNull(Packets.decodeScan(ByteArray(10)))
        assertNull(Packets.decodeScan("LDS9".toByteArray() + ByteArray(2000)))
        val b = ByteArray(26 + 720 * 3)
        b.putAscii(0, "LDS2")
        b.putU16(4, 720)
        // truncated: header claims 720 bins but no payload
        assertNull(Packets.decodeScan(b.copyOf(100)))
    }

    // ---- HACK / PNG2 / BYE1 ----------------------------------------------

    @Test
    fun hackDecodes() {
        val b = ByteArray(52)
        b.putAscii(0, "HACK")
        b.putU16(4, 2)        // proto
        b[6] = 0              // granted
        b[7] = 1              // S3
        b.putU16(8, 360)
        b.putU16(10, 90)
        b.putU32(12, 0xDEADBEEFL)
        b.putU32(16, 555L)
        val fw = "vacuum-phone-0.1.0"
        fw.toByteArray().copyInto(b, 20)
        b[20 + fw.length] = 0
        val h = Packets.decodeHack(b)!!
        assertTrue(h.granted)
        assertEquals(2, h.proto)
        assertEquals(1, h.boardType)
        assertEquals(360, h.scanBins)
        assertEquals(90, h.ticksPerRev)
        assertEquals(0xDEADBEEFL, h.session)
        assertEquals(555L, h.boardMs)
        assertEquals(fw, h.fwVersion)
    }

    @Test
    fun ping2AndByeDecode() {
        val p2 = ByteArray(20).also {
            it.putAscii(0, "PNG2")
            it.putU32(4, 9L)
            it.putU32(8, 1000L)
            it.putU32(12, 1015L)
            it.putU32(16, 1015_000L)
        }
        val ping = Packets.decodePing2(p2)!!
        assertEquals(9L, ping.seq)
        assertEquals(1000L, ping.phoneMs)
        assertEquals(1015L, ping.espMs)
        assertEquals(1015_000L, ping.espUs)

        val bye = ByteArray(5).also {
            it.putAscii(0, "BYE1")
            it[4] = 2
        }
        assertEquals(2, Packets.decodeBye(bye)!!.reason)
        assertNull(Packets.decodePing2(ByteArray(4)))
        assertNull(Packets.decodeBye(ByteArray(4)))
    }

    // ---- builders (phone -> board) ---------------------------------------

    @Test
    fun hbc2BuilderMatchesWireFormat() {
        val pkt = Packets.buildHbc2(seq = 77, rpmLx10 = -300, rpmRx10 = 300,
            flags = 0x0005, phoneMs = 123456L)
        assertEquals(16, pkt.size)
        assertArrayEquals("HBC2".toByteArray(), pkt.copyOf(4))
        assertEquals(77, (pkt[4].toInt() and 0xFF) or ((pkt[5].toInt() and 0xFF) shl 8))
        // -300 as i16 little-endian = 0xFED4
        assertEquals(0xD4, pkt[6].toInt() and 0xFF)
        assertEquals(0xFE, pkt[7].toInt() and 0xFF)
        assertEquals(0x0005, (pkt[10].toInt() and 0xFF) or ((pkt[11].toInt() and 0xFF) shl 8))
        // phone_ms 123456 little-endian at offset 12
        assertEquals(123456L, (pkt[12].toLong() and 0xFF) or
            ((pkt[13].toLong() and 0xFF) shl 8) or
            ((pkt[14].toLong() and 0xFF) shl 16) or
            ((pkt[15].toLong() and 0xFF) shl 24))
    }

    @Test
    fun heloBuilderMatchesWireFormat() {
        val pkt = Packets.buildHelo(2, 9900, 9902, takeover = true, session = 0xCAFEL)
        assertEquals(16, pkt.size)
        assertArrayEquals("HELO".toByteArray(), pkt.copyOf(4))
        assertEquals(2, (pkt[4].toInt() and 0xFF) or ((pkt[5].toInt() and 0xFF) shl 8))
        assertEquals(9900, (pkt[6].toInt() and 0xFF) or ((pkt[7].toInt() and 0xFF) shl 8))
        assertEquals(9902, (pkt[8].toInt() and 0xFF) or ((pkt[9].toInt() and 0xFF) shl 8))
        assertEquals(1, pkt[10].toInt() and 0xFF)   // takeover flag
        assertEquals(0xCAFEL, (pkt[12].toLong() and 0xFF) or
            ((pkt[13].toLong() and 0xFF) shl 8) or
            ((pkt[14].toLong() and 0xFF) shl 16) or
            ((pkt[15].toLong() and 0xFF) shl 24))
    }

    @Test
    fun ping1BuilderMatchesWireFormat() {
        val pkt = Packets.buildPing1(5, 999)
        assertEquals(12, pkt.size)
        assertArrayEquals("PNG1".toByteArray(), pkt.copyOf(4))
    }

    // ---- helpers ----------------------------------------------------------

    private fun ByteArray.putAscii(off: Int, s: String) {
        s.toByteArray(Charsets.US_ASCII).copyInto(this, off)
    }

    private fun ByteArray.putU16(off: Int, v: Int) {
        this[off] = (v and 0xFF).toByte()
        this[off + 1] = ((v shr 8) and 0xFF).toByte()
    }

    private fun ByteArray.putU32(off: Int, v: Long) {
        this[off] = (v and 0xFF).toByte()
        this[off + 1] = ((v shr 8) and 0xFF).toByte()
        this[off + 2] = ((v shr 16) and 0xFF).toByte()
        this[off + 3] = ((v shr 24) and 0xFF).toByte()
    }
}
