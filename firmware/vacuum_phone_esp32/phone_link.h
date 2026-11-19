// phone_link.h - protocol v2 between this board and the phone brain.
// Byte tables: docs/PROTOCOL.md. Replaces hb_link.h (HBT1/HBC1 to a fixed
// compile-time PC address) with owner claim + lease + clock sync:
//
//   HELO/HACK   phone claims ownership; a second phone is denied unless it
//               sets the takeover flag (two phones driving = dangerous)
//   PNG1/PNG2   NTP-style clock sync + keepalive (phone sends PNG1 >= 2 Hz
//               while idle; the board replies to the source port so the RTT
//               is meaningful)
//   HBC2        wheel commands ~20 Hz from the owner (legacy HBC1 still
//               accepted from the owner for one transition period)
//   HBT2        50 Hz telemetry, 96 B: HBT1 bytes 0-77 (magic changed) plus
//               command echo/latency, link flags, RSSI, sonar-reflex state,
//               lidar rotation, post-reflex output RPM, lidar CRC errors
//   BYE1        owner released / lease expired / takeover
//
// Lease: the owner is forgotten PH_LEASE_TIMEOUT_MS (3 s) after its last
// datagram of ANY kind - PNG1 is the keepalive, HBC2 traffic also refreshes
// it. With no owner the board sends nothing at all.
//
// E-stop: latched by HBC2 flag bit2, cleared by bit3 from the owner only.
// It survives lease expiry and holds the wheels at zero for EVERY source
// (phone, web stick, reflex) - see phApplyReflex().
//
// Task context: everything here runs in hoverboardTask on core 0. The owner
// IP/ports are also read by the scan sender (sendPendingScan, core 1), so
// they live behind a portMUX spinlock: IPAddress is not atomic.
//
// Little-endian both ways, same as v1.

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "config.h"
#include "hoverboard.h"
#include "sonar.h"
#include "lights.h"
#include "mpu9250.h"

HardwareSerial HbSerial(2);
Hoverboard     hoverboard;

#include "battery.h"     // after hoverboard.h: low-battery protection
#include "ota.h"         // ota_active / ota_robot_still
#include "web_drive.h"   // after hoverboard.h: the drive page (web_drive.h)

// Provided by lidar_ld14.h, which is included AFTER this file by the sketch
// (it needs phOwnerScanTarget(); we need its rate/CRC for HBT2 bytes 89/95).
float    phLidarFreqHz();
uint32_t phLidarCrcErrors();

// The UDP socket lives above the transports block: phSendTo() needs it.
static WiFiUDP  ph_udp;
static bool     ph_udp_open = false;

// ------------------------------------------------------------ transports
// A reply destination: UDP (the Wi-Fi link) or BLE (the FriendBot-style
// Bluetooth link, ble_link.h). Every reply (HACK/PNG2/HBT2/LDS2/BYE1) goes
// through phSendTo(); the owner may be on either transport.
struct PhTarget {
  bool ble = false;
  IPAddress ip;
  uint16_t port = 0;   // UDP only; BLE replies are GATT notifications
};

// Hooks installed by ble_link.h (null until the BLE stack starts).
static bool (*ph_ble_notify)(const uint8_t * buf, uint16_t len) = nullptr;
static bool (*ph_ble_connected_fn)() = nullptr;

static bool phBleClientConnected() {
  return ph_ble_connected_fn && ph_ble_connected_fn();
}

// BLE TX framing (Kotlin BleRobotLink parses this): a notify is either
//   [0x00][datagram...]                     - one whole packet, or
//   [0x01][idx][count][chunk...]            - LDS2 split for the MTU.
static const uint8_t PH_BLE_FRAME_SINGLE = 0x00;
static const uint8_t PH_BLE_FRAME_CHUNK  = 0x01;
static const uint16_t PH_BLE_CHUNK_PAYLOAD = 470;

static bool phSendTo(const PhTarget & t, const uint8_t * buf, uint16_t len) {
  if (t.ble) {
    if (!ph_ble_notify) return false;
    if (len <= PH_BLE_CHUNK_PAYLOAD + 1) {
      uint8_t frame[PH_BLE_CHUNK_PAYLOAD + 1];
      frame[0] = PH_BLE_FRAME_SINGLE;
      memcpy(frame + 1, buf, len);
      return ph_ble_notify(frame, (uint16_t)(len + 1));
    }
    // Chunked (LDS2): [0x01][idx][count][slice].
    const uint16_t count =
        (uint16_t)((len + PH_BLE_CHUNK_PAYLOAD - 1) / PH_BLE_CHUNK_PAYLOAD);
    for (uint16_t idx = 0; idx < count; ++idx) {
      const uint16_t off = idx * PH_BLE_CHUNK_PAYLOAD;
      uint16_t n = len - off;
      if (n > PH_BLE_CHUNK_PAYLOAD) n = PH_BLE_CHUNK_PAYLOAD;
      uint8_t frame[PH_BLE_CHUNK_PAYLOAD + 3];
      frame[0] = PH_BLE_FRAME_CHUNK;
      frame[1] = (uint8_t)idx;
      frame[2] = (uint8_t)count;
      memcpy(frame + 3, buf + off, n);
      if (!ph_ble_notify(frame, (uint16_t)(n + 3))) return false;
    }
    return true;
  }
  if (!ph_udp_open) return false;
  if (ph_udp.beginPacket(t.ip, t.port) == 1) {
    ph_udp.write(buf, len);
    return ph_udp.endPacket() == 1;
  }
  return false;
}

// ------------------------------------------------------------------- sizes
static const uint16_t HBT2_LEN = 96;   // HBT1's 78 +18 extension bytes
static const uint16_t HBC2_LEN = 16;
static const uint16_t HBC1_LEN = 12;   // legacy, still accepted
static const uint16_t HELO_LEN = 16;
static const uint16_t PNG1_LEN = 12;
static const uint16_t PNG2_LEN = 20;
static const uint16_t BYE1_LEN = 5;

// HBT1 flags (bytes 10-11) - unchanged meaning, carried in HBT2 too.
static const uint16_t HBT_MAINBOARD_UP  = 0x0001;
static const uint16_t HBT_CMD_TIMEOUT   = 0x0002;
static const uint16_t HBT_CURRENT_VALID = 0x0004;
static const uint16_t HBT_SONAR_NEAR    = 0x0008;
static const uint16_t HBT_SONAR_BACKOFF = 0x0010;
static const uint16_t HBT_SONAR_STALE   = 0x0020;
static const uint16_t HBT_TICKS_SYNTH   = 0x0040;
static const uint16_t HBT_BUZZER        = 0x0080;
static const uint16_t HBT_SONAR_L_NEAR  = 0x0100;
static const uint16_t HBT_SONAR_R_NEAR  = 0x0200;
static const uint16_t HBT_SONAR_TURNING = 0x0400;
static const uint16_t HBT_SONAR_L_STALE = 0x0800;
static const uint16_t HBT_SONAR_R_STALE = 0x1000;
static const uint16_t HBT_LIGHT_LEFT    = 0x2000;
static const uint16_t HBT_LIGHT_RIGHT   = 0x4000;
static const uint16_t HBT_LIGHT_BACK    = 0x8000;

// HBC2 flags (docs/PROTOCOL.md §3.4).
static const uint16_t HBC2_ENABLE       = 0x0001;
static const uint16_t HBC2_BUZZER       = 0x0002;
static const uint16_t HBC2_ESTOP        = 0x0004;  // latch
static const uint16_t HBC2_ESTOP_CLEAR  = 0x0008;
// HBC1 legacy flags.
static const uint16_t HBC_ENABLE        = 0x0001;
static const uint16_t HBC_BUZZER        = 0x0002;

// HELO flags.
static const uint16_t HELO_TAKEOVER     = 0x0001;
// HACK status.
static const uint8_t  HACK_GRANTED      = 0;
static const uint8_t  HACK_DENIED       = 1;
static const uint8_t  HACK_VERSION      = 2;
// BYE1 reasons.
static const uint8_t  BYE_RELEASED      = 0;
static const uint8_t  BYE_LEASE_EXPIRED = 1;
static const uint8_t  BYE_TAKEOVER      = 2;

static bool hb_buzzer = false;        // what the pin is doing
static bool hb_buzzer_phone = false;  // what HBC2/HBC1 last asked for

static void hbSetBuzzer(bool on) {
  if (on == hb_buzzer) return;
  hb_buzzer = on;
  digitalWrite(BUZZER_PIN, (on == BUZZER_ACTIVE_HIGH) ? HIGH : LOW);
}

// ------------------------------------------------------------------- socket
static uint8_t  ph_rx_buf[64];        // HELO/HBC2/PNG1/HBC1 all fit
static uint8_t  ph_tx_buf[HBT2_LEN];

// Owner state - written by hoverboardTask (core 0), read by the scan sender
// (core 1) through phOwnerScanTarget(). portMUX because IPAddress (4 bytes,
// but a struct copy) is not atomic against a concurrent claim.
static portMUX_TYPE ph_owner_mux = portMUX_INITIALIZER_UNLOCKED;
static PhTarget ph_owner;                  // identity (UDP addr or BLE)
static uint16_t  ph_owner_telem_port = 0;  // HBT2 destination (UDP only)
static uint16_t  ph_owner_scan_port = 0;   // LDS2 destination (UDP only)
static uint32_t  ph_session = 0;
static uint32_t  ph_lease_deadline = 0;    // millis() after which we forget
static bool      ph_owned = false;

// Command state (echo fields for HBT2).
static uint16_t ph_telem_seq = 0;        // telemetry sequence (HBT1 byte 4)
static uint16_t ph_last_cmd_seq = 0;
static uint32_t ph_last_cmd_phone_ms = 0;
static uint32_t ph_last_cmd_ms = 0;        // board millis of last accepted cmd
static uint32_t ph_cmd_count = 0;
static uint32_t ph_cmd_bad = 0;
static bool     ph_estop = false;          // latched; survives lease expiry

static inline void put16(uint8_t * b, uint16_t off, uint16_t v) {
  b[off]     = (uint8_t)v;
  b[off + 1] = (uint8_t)(v >> 8);
}

static inline void put32(uint8_t * b, uint16_t off, uint32_t v) {
  b[off]     = (uint8_t)v;
  b[off + 1] = (uint8_t)(v >> 8);
  b[off + 2] = (uint8_t)(v >> 16);
  b[off + 3] = (uint8_t)(v >> 24);
}

static inline uint16_t get16(const uint8_t * b, uint16_t off) {
  return (uint16_t)b[off] | ((uint16_t)b[off + 1] << 8);
}

static inline int16_t get16s(const uint8_t * b, uint16_t off) {
  return (int16_t)get16(b, off);
}

static inline uint32_t get32(const uint8_t * b, uint16_t off) {
  return (uint32_t)b[off] | ((uint32_t)b[off + 1] << 8) |
         ((uint32_t)b[off + 2] << 16) | ((uint32_t)b[off + 3] << 24);
}

static inline uint16_t sat16(float v) {
  return (uint16_t)(int16_t)constrain(lroundf(v), -32768L, 32767L);
}

static bool phFromOwner(const IPAddress & ip) {
  return ph_owned && !ph_owner.ble && ip == ph_owner.ip;
}
static bool phFromOwner(const PhTarget & t) {
  if (!ph_owned) return false;
  if (t.ble) return ph_owner.ble;
  return !ph_owner.ble && t.ip == ph_owner.ip;
}

static void phClaim(const PhTarget & target, uint16_t telem_port,
                    uint16_t scan_port, uint32_t session) {
  portENTER_CRITICAL(&ph_owner_mux);
  ph_owner = target;
  ph_owner_telem_port = telem_port;
  ph_owner_scan_port = scan_port;
  ph_session = session;
  ph_owned = true;
  ph_lease_deadline = millis() + PH_LEASE_TIMEOUT_MS;
  portEXIT_CRITICAL(&ph_owner_mux);
}

// Send BYE1 to the current owner and forget it. The E-stop latch is NOT
// cleared here (prompt §4.4: it survives lease expiry).
static void phRelease(uint8_t reason) {
  PhTarget old;
  uint16_t old_port;
  portENTER_CRITICAL(&ph_owner_mux);
  old = ph_owner;
  old_port = ph_owner_telem_port;
  ph_owned = false;
  ph_owner = PhTarget();
  ph_owner_telem_port = 0;
  ph_owner_scan_port = 0;
  portEXIT_CRITICAL(&ph_owner_mux);

  if (ph_owned || old.ble || old.ip != IPAddress()) {
    ph_tx_buf[0] = 'B'; ph_tx_buf[1] = 'Y'; ph_tx_buf[2] = 'E'; ph_tx_buf[3] = '1';
    ph_tx_buf[4] = reason;
    PhTarget to = old;
    if (!to.ble) to.port = old_port;
    phSendTo(to, ph_tx_buf, BYE1_LEN);
  }
  Serial.printf("phone_link: owner released (reason %u)\n", reason);
}

// Scan sender (core 1) asks where the latest scan goes.
static bool phOwnerScanTarget(PhTarget & out) {
  bool ok;
  portENTER_CRITICAL(&ph_owner_mux);
  ok = ph_owned;
  out = ph_owner;
  if (!out.ble) out.port = ph_owner_scan_port;
  portEXIT_CRITICAL(&ph_owner_mux);
  return ok && (out.ble || out.port != 0);
}

static bool phEstopLatched() { return ph_estop; }

// Link flags byte for HBT2 byte 86: bit0 estop latched, bit1 web stick owns,
// bit2 OTA active, bit3 AP mode, bit4 STA connected.
static uint8_t phLinkFlags() {
  uint8_t f = 0;
  if (ph_estop)          f |= 0x01;
  if (webDriveOwns())    f |= 0x02;
  if (ota_active)        f |= 0x04;
  if (wifi_ap_up)        f |= 0x08;
  if (WiFi.status() == WL_CONNECTED) f |= 0x10;
  return f;
}

// Sonar-reflex state byte for HBT2 byte 88: 0 none, 1 stop (forward
// stripped), 2 back-off, 3 turn. See sonar.h (SONAR_REFLEX_MODE).
static uint8_t phSonarReflexByte() {
  switch (sonar_reflex) {
    case REFLEX_STOP:  return 1;
    case REFLEX_BACK:  return 2;
    case REFLEX_TURN_LEFT:
    case REFLEX_TURN_RIGHT: return 3;
    default:           return 0;
  }
}

// ----------------------------------------------------------------- HELO/HACK
static void phSendHack(const PhTarget & to, uint8_t status,
                       uint32_t session) {
  ph_tx_buf[0] = 'H'; ph_tx_buf[1] = 'A'; ph_tx_buf[2] = 'C'; ph_tx_buf[3] = 'K';
  put16(ph_tx_buf, 4, PH_PROTO_VERSION);
  ph_tx_buf[6] = status;
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  ph_tx_buf[7] = 1;   // board type: ESP32-S3
#else
  ph_tx_buf[7] = 0;   // board type: ESP32-WROOM
#endif
  put16(ph_tx_buf, 8, SCAN_BINS);
  put16(ph_tx_buf, 10, (uint16_t)HB_TICKS_PER_REV);
  put32(ph_tx_buf, 12, session);
  put32(ph_tx_buf, 16, millis());
  // fw version: NUL-terminated ASCII, <= 32 bytes (bytes 20..51).
  size_t n = strnlen(FW_VERSION, 31);
  memcpy(ph_tx_buf + 20, FW_VERSION, n);
  ph_tx_buf[20 + n] = '\0';
  const uint16_t len = (uint16_t)(20 + n + 1);
  phSendTo(to, ph_tx_buf, len);
}

// ------------------------------------------------------------------- PNG1/2
static void phSendPing2(const PhTarget & to, const uint8_t * rx) {
  ph_tx_buf[0] = 'P'; ph_tx_buf[1] = 'N'; ph_tx_buf[2] = 'G'; ph_tx_buf[3] = '2';
  put32(ph_tx_buf, 4, get32(rx, 4));    // seq echo
  put32(ph_tx_buf, 8, get32(rx, 8));    // phone_ms echo
  put32(ph_tx_buf, 12, millis());       // esp_ms
  put32(ph_tx_buf, 16, micros());       // esp_us (finer RTT)
  phSendTo(to, ph_tx_buf, PNG2_LEN);
}

// -------------------------------------------------------------- HBT2 telemetry
static void phSendTelemetry() {
  if (!ph_owned || !ph_udp_open)
    return;   // no owner: the board sends nothing (prompt §4.3)

  uint16_t flags = 0;
  if (hoverboard.isConnected())        flags |= HBT_MAINBOARD_UP;
  if (hoverboard.isTimedOut())         flags |= HBT_CMD_TIMEOUT;
  if (hoverboard.hasCurrentFeedback()) flags |= HBT_CURRENT_VALID;
  if (sonar.obstacleNear())            flags |= HBT_SONAR_NEAR;
  if (sonarBackingOff())               flags |= HBT_SONAR_BACKOFF;
  if (sonarLeft.obstacleNear())        flags |= HBT_SONAR_L_NEAR;
  if (sonarRight.obstacleNear())       flags |= HBT_SONAR_R_NEAR;
  if (sonarTurning())                  flags |= HBT_SONAR_TURNING;
  if (sonarLeft.isStale())             flags |= HBT_SONAR_L_STALE;
  if (sonarRight.isStale())            flags |= HBT_SONAR_R_STALE;
  if (light_left_on)                   flags |= HBT_LIGHT_LEFT;
  if (light_right_on)                  flags |= HBT_LIGHT_RIGHT;
  if (light_back_on)                   flags |= HBT_LIGHT_BACK;
  if (sonar.isStale())                 flags |= HBT_SONAR_STALE;
  if (hoverboard.getFeedbackBytes() == HB_FB_LEN_UPSTREAM)
    flags |= HBT_TICKS_SYNTH;
  if (hb_buzzer)                       flags |= HBT_BUZZER;

  // Bytes 0-77: identical to HBT1 except the magic (docs/PROTOCOL.md §3.6).
  // Byte 4 stays the TELEMETRY sequence (HBT1's seq); the command echo lives
  // at byte 78 in the extension.
  ph_tx_buf[0] = 'H'; ph_tx_buf[1] = 'B'; ph_tx_buf[2] = 'T'; ph_tx_buf[3] = '2';
  put16(ph_tx_buf, 4,  ph_telem_seq++);
  put32(ph_tx_buf, 6,  millis());
  put16(ph_tx_buf, 10, flags);
  put16(ph_tx_buf, 12, hoverboard.getFeedbackBytes());
  put32(ph_tx_buf, 14, (uint32_t)hoverboard.getWheelTicks(HB_LEFT));
  put32(ph_tx_buf, 18, (uint32_t)hoverboard.getWheelTicks(HB_RIGHT));
  put16(ph_tx_buf, 22, sat16(hoverboard.getWheelRPM(HB_LEFT)));
  put16(ph_tx_buf, 24, sat16(hoverboard.getWheelRPM(HB_RIGHT)));
  put16(ph_tx_buf, 26, sat16(hoverboard.getBatteryVolts() * 100.0f));
  put16(ph_tx_buf, 28, sat16(hoverboard.getBoardTempC() * 10.0f));
  put16(ph_tx_buf, 30, sat16(hoverboard.getCurrentAmps(HB_LEFT) * 100.0f));
  put16(ph_tx_buf, 32, sat16(hoverboard.getCurrentAmps(HB_RIGHT) * 100.0f));
  put32(ph_tx_buf, 34, hoverboard.getFrameCount());
  put32(ph_tx_buf, 38, hoverboard.getChecksumErrorCount());
  put16(ph_tx_buf, 42, sonar.distanceMm());
  put16(ph_tx_buf, 44, (uint16_t)hoverboard.getCmdSteer());
  put16(ph_tx_buf, 46, (uint16_t)hoverboard.getCmdSpeed());
  put16(ph_tx_buf, 48, sonarLeft.distanceMm());
  put16(ph_tx_buf, 50, sonarRight.distanceMm());
  put16(ph_tx_buf, 52, imuStatus());
  for (int i = 0; i < 4; i++) put16(ph_tx_buf, 54 + 2 * i, (uint16_t)imu_sample.quat[i]);
  for (int i = 0; i < 3; i++) put16(ph_tx_buf, 62 + 2 * i, (uint16_t)imu_sample.gyro[i]);
  for (int i = 0; i < 3; i++) put16(ph_tx_buf, 68 + 2 * i, (uint16_t)imu_sample.accel[i]);
  ph_tx_buf[74] = (uint8_t)bat_state;
  ph_tx_buf[75] = (uint8_t)BAT_CELLS;
  put16(ph_tx_buf, 76, (uint16_t)constrain(lroundf(bat_vf * 100.0f), 0L, 65535L));

  // ---- extension bytes 78-95 (prompt §4.6 / docs/PROTOCOL.md §3.6) ----
  put16(ph_tx_buf, 78, ph_last_cmd_seq);
  put32(ph_tx_buf, 80, ph_last_cmd_phone_ms);           // phone_ms echo
  const uint32_t age = (ph_last_cmd_ms == 0 || millis() < ph_last_cmd_ms)
                           ? 0xFFFFu
                           : millis() - ph_last_cmd_ms;
  put16(ph_tx_buf, 84, (uint16_t)constrain((long)age, 0L, 0xFFFFL));
  ph_tx_buf[86] = phLinkFlags();
  ph_tx_buf[87] = (WiFi.status() == WL_CONNECTED)
                      ? (uint8_t)(int8_t)WiFi.RSSI()
                      : 0;
  ph_tx_buf[88] = phSonarReflexByte();
  const float hz = phLidarFreqHz();
  const uint16_t deci = (uint16_t)constrain(lroundf(hz * 10.0f), 0L, 255L);
  ph_tx_buf[89] = (uint8_t)deci;
  put16(ph_tx_buf, 90, sat16(hoverboard.getOutputRPM(HB_LEFT) * 10.0f));
  put16(ph_tx_buf, 92, sat16(hoverboard.getOutputRPM(HB_RIGHT) * 10.0f));
  put16(ph_tx_buf, 94, (uint16_t)phLidarCrcErrors());

  // Destination: the owner (UDP telem port or BLE notify).
  PhTarget to;
  portENTER_CRITICAL(&ph_owner_mux);
  to = ph_owner;
  if (!to.ble) to.port = ph_owner_telem_port;
  portEXIT_CRITICAL(&ph_owner_mux);
  if (!to.ble && to.port == 0) return;
  phSendTo(to, ph_tx_buf, HBT2_LEN);
}

// ------------------------------------------------------------ receive + lease
// Drain the socket, act on the newest valid datagram of each kind. ANY
// datagram from the owner refreshes the lease (prompt §4.3).
// One datagram from either transport. `from` is where replies go (UDP
// source, or BLE notify). Shared by phPoll() (UDP) and ble_link.h (GATT).
static void phHandleDatagram(const uint8_t * d, int n, const PhTarget & from) {
  if (n < 4)
    return;
  const bool from_owner = phFromOwner(from);

  if (from_owner)
    ph_lease_deadline = millis() + PH_LEASE_TIMEOUT_MS;

  if (n >= (int)HELO_LEN && memcmp(d, "HELO", 4) == 0) {
    const uint16_t proto = get16(d, 4);
    const uint16_t scan_port = get16(d, 6);
    const uint16_t telem_port = get16(d, 8);
    const uint16_t hflags = get16(d, 10);
    const uint32_t session = get32(d, 12);

    uint8_t status;
    if (proto != PH_PROTO_VERSION) {
      status = HACK_VERSION;
    } else if (!ph_owned || (hflags & HELO_TAKEOVER) ||
               (from_owner && session == ph_session)) {
      if (ph_owned && !from_owner) {
        // Stealing a live owner: tell the old phone first.
        phRelease(BYE_TAKEOVER);
      }
      phClaim(from, telem_port, scan_port, session);
      status = HACK_GRANTED;
      Serial.printf("phone_link: owner %s claimed (telem %u, scan %u, "
                    "session %08lx)%s\n",
                    from.ble ? "BLE" : from.ip.toString().c_str(),
                    telem_port, scan_port, (unsigned long)session,
                    from.ble ? " [bluetooth]" : "");
    } else {
      status = HACK_DENIED;
      Serial.printf("phone_link: HELO from %s denied (owner present)\n",
                    from.ble ? "BLE" : from.ip.toString().c_str());
    }
    phSendHack(from, status, session);
    return;
  }

  if (n >= (int)PNG1_LEN && memcmp(d, "PNG1", 4) == 0) {
    if (from_owner) {
      phSendPing2(from, d);  // reply to the source (UDP port / BLE notify)
      ph_lease_deadline = millis() + PH_LEASE_TIMEOUT_MS;
    }
    return;
  }

  if (n >= (int)HBC2_LEN && memcmp(d, "HBC2", 4) == 0) {
    if (!from_owner) { ph_cmd_bad++; return; }
    const uint16_t seq = get16(d, 4);
    const float want_l = get16s(d, 6) / 10.0f;
    const float want_r = get16s(d, 8) / 10.0f;
    const uint16_t cflags = get16(d, 10);
    const uint32_t phone_ms = get32(d, 12);

    if (cflags & HBC2_ESTOP) {
      if (!ph_estop) {
        ph_estop = true;
        hoverboard.setWheelRPM(0.0f, 0.0f);  // latch with the wheels already zero
        Serial.println("phone_link: E-STOP latched");
      }
    }
    if (cflags & HBC2_ESTOP_CLEAR) {
      if (ph_estop) {
        ph_estop = false;
        Serial.println("phone_link: E-STOP cleared");
      }
    }

    ph_last_cmd_seq = seq;
    ph_last_cmd_phone_ms = phone_ms;
    ph_last_cmd_ms = millis();
    ph_cmd_count++;

    if (ph_estop) {
      // Latched: every source is held at zero until the owner clears.
      hoverboard.setWheelRPM(0.0f, 0.0f);
    } else if (cflags & HBC2_ENABLE) {
      if (!webDriveOwns()) hoverboard.setWheelRPM(want_l, want_r);
    } else {
      // Explicit stop (bit0 clear): still refreshes the timeout, like HBC1.
      if (!webDriveOwns()) hoverboard.setWheelRPM(0.0f, 0.0f);
    }
    hb_buzzer_phone = (cflags & HBC2_BUZZER) != 0;
    return;
  }

  // Legacy HBC1 from the owner only (one transition period, prompt §4.2).
  if (n >= (int)HBC1_LEN && memcmp(d, "HBC1", 4) == 0) {
    if (!from_owner) { ph_cmd_bad++; return; }
    const float want_l = get16s(d, 6);
    const float want_r = get16s(d, 8);
    const uint16_t cflags = get16(d, 10);
    ph_last_cmd_seq = get16(d, 4);
    ph_last_cmd_phone_ms = 0;
    ph_last_cmd_ms = millis();
    ph_cmd_count++;
    if (ph_estop) {
      hoverboard.setWheelRPM(0.0f, 0.0f);
    } else if (!webDriveOwns()) {
      hoverboard.setWheelRPM((cflags & HBC_ENABLE) ? want_l : 0.0f,
                             (cflags & HBC_ENABLE) ? want_r : 0.0f);
    }
    hb_buzzer_phone = (cflags & HBC_BUZZER) != 0;
    return;
  }

  ph_cmd_bad++;
}

static void phPoll() {
  if (!ph_udp_open)
    return;

  while (ph_udp.parsePacket() > 0) {
    const int n = ph_udp.read(ph_rx_buf, sizeof(ph_rx_buf));
    PhTarget from;
    from.ble = false;
    from.ip = ph_udp.remoteIP();
    from.port = ph_udp.remotePort();
    phHandleDatagram(ph_rx_buf, n, from);
  }

  // Lease expiry: forget the owner, wheels to zero (BYE1 goes out first).
  if (ph_owned && (int32_t)(millis() - ph_lease_deadline) >= 0) {
    hoverboard.setWheelRPM(0.0f, 0.0f);
    phRelease(BYE_LEASE_EXPIRED);
  }
}

// BLE entry point (ble_link.h): one GATT write = one datagram.
static void phHandleBle(const uint8_t * data, int n) {
  PhTarget from;
  from.ble = true;
  phHandleDatagram(data, n, from);
}

// ------------------------------------------------------------------- reflex
// The obstacle reflex and the E-stop sit between the network and the wheels,
// so they still work when WiFi drops or the phone crashes. Recomputed from
// the request every cycle rather than written back into it: an obstacle never
// waits for the next datagram, and a back-off stops the moment the sonar
// clears instead of lingering as the new setpoint.
//
// Only while something is asking for motion. With no request the wheels stay
// still: backing off is blind, and a robot that reverses by itself at
// power-on because something is near the sonar is worse than one that
// simply refuses to go forward.
static void phApplyReflex() {
  if (ota_active) {               // flashing: nothing moves until the reboot
    hoverboard.setOutputRPM(0.0f, 0.0f);
    sonarReflexIdle();
    return;
  }
  if (ph_estop) {                 // latched: EVERY source held at zero
    hoverboard.setOutputRPM(0.0f, 0.0f);
    sonarReflexIdle();
    return;
  }
  float l = hoverboard.getTargetRPM(HB_LEFT);
  float r = hoverboard.getTargetRPM(HB_RIGHT);
  if (l == 0.0f && r == 0.0f) {
    hoverboard.setOutputRPM(0.0f, 0.0f);
    sonarReflexIdle();
    return;
  }
  sonarGate(&l, &r);
  // Battery last, so it also limits the reflex: a critical pack stops even a
  // back-off (battery.h).
  const float k = batterySpeedScale();
  hoverboard.setOutputRPM(l * k, r * k);
}

// --------------------------------------------------------------------- log
static void phLog() {
  if (!hoverboard.isConnected()) {
    Serial.printf("hoverboard: no feedback on GPIO%d (%lu frames, %lu bad). "
                  "check TX/RX, common ground, %lu baud, VARIANT_USART "
                  "(USART2 = left)\n",
                  HB_RX_PIN, (unsigned long)hoverboard.getFrameCount(),
                  (unsigned long)hoverboard.getChecksumErrorCount(),
                  (unsigned long)HB_BAUD);
  } else {
    Serial.printf("hoverboard: %u B every %lu ms  %.1f V  %.0f C  ticks %ld/%ld  "
                  "rpm %.0f/%.0f  %lu cmd  %lu bad%s%s\n",
                  (unsigned)hoverboard.getFeedbackBytes(),
                  (unsigned long)hoverboard.getFeedbackPeriodMs(),
                  hoverboard.getBatteryVolts(), hoverboard.getBoardTempC(),
                  (long)hoverboard.getWheelTicks(HB_LEFT),
                  (long)hoverboard.getWheelTicks(HB_RIGHT),
                  hoverboard.getWheelRPM(HB_LEFT),
                  hoverboard.getWheelRPM(HB_RIGHT),
                  (unsigned long)ph_cmd_count, (unsigned long)ph_cmd_bad,
                  hoverboard.isTimedOut() ? "  [CMD TIMEOUT - stopped]" : "",
                  ph_estop ? "  [E-STOP latched]" : "");
  }

  Serial.printf("phone_link: %s  session %08lx  lease in %ld ms  "
                "estop %d  cmd age %lu ms\n",
                ph_owned ? "OWNED" : "no owner",
                (unsigned long)ph_session,
                ph_owned ? (long)(ph_lease_deadline - millis()) : -1L,
                ph_estop ? 1 : 0,
                ph_last_cmd_ms ? (unsigned long)(millis() - ph_last_cmd_ms) : 0UL);

  if (HB_SPEED_LOOP && hoverboard.isConnected()) {
    static const char * F[] = {"", " STALLED", " WRONG-WAY"};
    Serial.printf("speed loop: L tgt %.0f meas %.0f out %.0f%s | "
                  "R tgt %.0f meas %.0f out %.0f%s\n",
                  hoverboard.getOutputRPM(HB_LEFT), hoverboard.getWheelRPM(HB_LEFT),
                  hoverboard.getLoopOutput(HB_LEFT), F[hoverboard.getLoopFault(HB_LEFT)],
                  hoverboard.getOutputRPM(HB_RIGHT), hoverboard.getWheelRPM(HB_RIGHT),
                  hoverboard.getLoopOutput(HB_RIGHT), F[hoverboard.getLoopFault(HB_RIGHT)]);
  }

  const HCSR04 * const S[] = {&sonar, &sonarLeft, &sonarRight};
  const char * const NAME[] = {"front", "left", "right"};
  for (int i = 0; i < 3; i++) {
    Serial.printf("sonar %-5s: %u mm  %+.2f m/s  %s  %lu noise\n", NAME[i],
                  (unsigned)S[i]->distanceMm(), S[i]->speed(),
                  S[i]->isStale() ? "NO SENSOR - forward blocked" :
                    (S[i]->obstacleNear() ? "OBSTACLE" : "clear"),
                  (unsigned long)S[i]->noiseCount());
  }
  if (sonar_reflex != REFLEX_NONE)
    Serial.printf("sonar reflex: %s (mode %s)\n", sonarReflexName(),
#if SONAR_REFLEX_MODE == SONAR_REFLEX_STOP
                  "STOP"
#else
                  "FULL"
#endif
    );
  batteryLog();
  imuLog();
}

// --------------------------------------------------------------------- task
// Core 0: mainboard UART, sonar, IMU, phone socket, watchdog, reflex.
// Higher priority than lidarTask: a late command frame lets the mainboard
// hit its own serial timeout.
static void hoverboardTask(void * arg) {
  (void)arg;
  uint32_t last_telem_ms = 0;
  uint32_t last_imu_ms = 0;
  uint32_t last_log_ms = 0;
  uint32_t last_web_ms = 0;

  for (;;) {
    hoverboard.poll();    // drains the UART, enforces the command timeout
    sonarLoop();          // fires the next ping when due, collects echoes

    // Same rate as HBT2, but read with or without WiFi, so the boot log and
    // the IMU filter do not wait for the network.
    if (millis() - last_imu_ms >= PH_TELEM_PERIOD_MS) {
      last_imu_ms = millis();
      const bool still =
          hoverboard.getOutputRPM(HB_LEFT) == 0.0f &&
          hoverboard.getOutputRPM(HB_RIGHT) == 0.0f &&
          fabsf(hoverboard.getWheelRPM(HB_LEFT)) < 1.0f &&
          fabsf(hoverboard.getWheelRPM(HB_RIGHT)) < 1.0f;
      imuRead(still);
      ota_robot_still = still;    // OTA only answers while this holds
    }

    if (netUp()) {
      if (!ph_udp_open) {
        ph_udp.begin((uint16_t)PH_PORT_DEFAULT_CMD);
        ph_udp_open = true;
        Serial.printf("phone_link: listening on UDP %d\n", PH_PORT_DEFAULT_CMD);
      }
      phPoll();

      const uint32_t now = millis();
      if (now - last_telem_ms >= PH_TELEM_PERIOD_MS) {
        last_telem_ms = now;
        phSendTelemetry();
      }
    } else if (ph_udp_open) {
      // Hoverboard::poll()'s command timeout stops the wheels by itself once
      // datagrams stop; this only releases the socket and the owner.
      ph_udp.stop();
      ph_udp_open = false;
      if (ph_owned) phRelease(BYE_LEASE_EXPIRED);
      Serial.println("phone_link: network down, socket closed");
    }

    webDriveApply(hoverboard);   // web stick command, or hand back
    batteryUpdate(hoverboard);
    phApplyReflex();
    lightsUpdate(hoverboard);
    if (millis() - last_web_ms >= WEB_STATUS_PERIOD_MS) {
      last_web_ms = millis();
      webDrivePublish(hoverboard, sonar.obstacleNear(), sonarReflexName(),
                      sonar.distanceMm(), sonarLeft.distanceMm(), sonarRight.distanceMm());
    }
    // The phone's buzzer follows the wheels' rule (no commands, no noise);
    // the battery's chirps sound regardless.
    hbSetBuzzer(!ota_active && ((hb_buzzer_phone && !hoverboard.isTimedOut()) || batteryBeep()));
    hoverboard.transmit();

    const uint32_t now = millis();
    if (now - last_log_ms >= HB_LOG_PERIOD_MS) {
      last_log_ms = now;
      phLog();
    }

    vTaskDelay(1);
  }
}

static void phBegin() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_HIGH ? LOW : HIGH);   // off

  sonarBegin();
  Serial.printf("sonar  front TRIG=GPIO%d ECHO=GPIO%d, stop<%u mm, clear>%u mm; "
                "reflex mode %s\n",
                SONAR_TRIG_PIN, SONAR_ECHO_PIN, SONAR_STOP_MM, SONAR_CLEAR_MM,
#if SONAR_REFLEX_MODE == SONAR_REFLEX_STOP
                "STOP (forward stripped only)"
#else
                "FULL (back-off / turn-away)"
#endif
  );
  Serial.printf("sonar  left  TRIG=GPIO%d ECHO=GPIO%d, right TRIG=GPIO%d "
                "ECHO=GPIO%d, stop<%u mm, clear>%u mm\n",
                SONAR_LEFT_TRIG_PIN, SONAR_LEFT_ECHO_PIN, SONAR_RIGHT_TRIG_PIN,
                SONAR_RIGHT_ECHO_PIN, SONAR_SIDE_STOP_MM, SONAR_SIDE_CLEAR_MM);

  lightsBegin();
  Serial.printf("lights LED left=GPIO%d right=GPIO%d back=GPIO%d\n",
                LED_LEFT_PIN, LED_RIGHT_PIN, LED_BACK_PIN);

  imuBegin();     // ~50 ms power-on wait; before the tasks

  Serial.printf("battery %d cells: low <%.1f V (speed x%.1f), critical <%.1f V (stop), "
                "recover >%.1f V%s\n", (int)BAT_CELLS, BAT_LOW_V, BAT_LOW_SPEED_SCALE,
                BAT_CRIT_V, BAT_RECOVER_V, BAT_PROTECT_ENABLED ? "" : "  [DISABLED]");

  hoverboard.begin(&HbSerial);
  Serial.printf("hover  UART2 @ %lu baud, RX=GPIO%d TX=GPIO%d, "
                "%ld ticks/rev, N_MOT_MAX %.0f rpm, cmd timeout %lu ms\n",
                (unsigned long)HB_BAUD, HB_RX_PIN, HB_TX_PIN,
                (long)HB_TICKS_PER_REV, HB_MB_N_MOT_MAX,
                (unsigned long)HB_CMD_TIMEOUT_MS);

  Serial.printf("phone_link: proto v%u, cmd %u, telem %u, scan %u, lease %lu ms, "
                "fw %s\n",
                PH_PROTO_VERSION, PH_PORT_DEFAULT_CMD, PH_PORT_DEFAULT_TELEM,
                PH_PORT_DEFAULT_SCAN, (unsigned long)PH_LEASE_TIMEOUT_MS,
                FW_VERSION);
}
