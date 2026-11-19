# VacuumBot protocol — phone ⇄ ESP32

Everything on the wire between the Android phone and the ESP32, in one place.
Two protocol generations:

- **v1 (legacy)** — the current `firmware/ldrobot_ld14p_esp32_mpu9250_ros`
  wire format (LDS1 / HBT1 / HBC1), verified byte-for-byte against both the
  firmware sender and the reference decoder in
  `robot_ws/src/esp_tcp_server` (`hoverboard_udp_bridge.cpp`,
  `udp_lidar_receiver.cpp`). The new firmware keeps receiving HBC1 and the
  app decoders are built from captured v1 packets, so v1 stays documented
  here forever.
- **v2 (phone brain)** — adds owner claim/lease, clock sync, echo/latency
  fields, scan intensity and deskew timestamps. Designed in
  `docs/ASSUMPTIONS.md` / the build prompt; implemented by
  `firmware/vacuum_phone_esp32` and the app's `RobotLink`.

Conventions for **all** packets:

- Transport: UDP, one datagram per message. No fragmentation: every packet
  fits one MTU (≤ 1472 B payload).
- **Little-endian** for every multi-byte field, both directions (v1 firmware
  comment: "Little-endian both ways").
- Magic = 4 ASCII bytes at offset 0. Receivers dispatch on magic, then check
  length. Unknown magic → drop, count as `bad`.
- Multi-byte integers are two's complement where signed.
- Angles: degrees in packets (lidar), radians in the app core (SI).
- Times: `*_ms` fields are `millis()`-style wrap-around 32-bit millisecond
  counters of the sending machine. The phone stamps arrival with
  `SystemClock.elapsedRealtimeNanos()` and reconciles board time via PNG1/PNG2.

## 1. Ports

| Port | Proto | Direction | Content |
|------|-------|-----------|---------|
| 9900 | v1/v2 | ESP32 → phone | LDS1 / LDS2 lidar scans |
| 9901 | v1/v2 | phone → ESP32 | HBC1 / HBC2 commands, HELO, PNG1, BYE1 (phone→board) |
| 9902 | v1/v2 | ESP32 → phone | HBT1 / HBT2 telemetry, PNG2 |

Defaults from `config.h` (`DEST_PORT 9900`, `HB_CMD_PORT 9901`,
`HB_TELEM_PORT 9902`). In v2 the phone **announces its own ports** in HELO
(`scan_port`, `telem_port`) and the board sends scans/telemetry there — the
defaults above are what the phone asks for.

v1 sends everything to a fixed `DEST_IP` ("192.168.1.12", compile-time). v2
sends only to the claimed owner.

---

## 2. Protocol v1 (legacy — verified against repo)

### 2.1 LDS1 — lidar scan, ESP32 → phone, UDP 9900, one datagram per revolution

Source of truth: `lidar_ld14.h` sender + `udp_lidar_receiver.cpp` decoder.
360 bins → 14 + 720 = **734 B**.

| Offset | Size | Field | Type | Notes |
|--------|------|-------|------|-------|
| 0 | 4 | magic | ASCII | `"LDS1"` |
| 4 | 2 | bins | u16 | `SCAN_BINS`, 360 default, ≤ 720 |
| 6 | 4 | scan_id | u32 | increments per sent scan |
| 10 | 4 | reserved | — | zero; receiver skips bytes 10..13 |
| 14 | 2·bins | ranges | u16 LE | millimetres; **0 = no return** (receiver publishes +inf) |

Semantics:

- **Bin `i` is the measurement `i·(360/bins)` degrees CLOCKWISE from the
  lidar's own zero mark.** The array is never reordered on the board; the
  phone applies mirror + mounting yaw itself (v1 receiver used
  `flip_lr=true`, `angle0_deg=180`).
- Ranges are clipped on the board to `RANGE_MIN_MM=20 … RANGE_MAX_MM=8000`.
- Per-point ranging-centre correction (LDROBOT SDK `OFFSET_X=5.9`,
  `OFFSET_Y=-18.975571` mm) is already applied; angles are corrected too.
- Several points per bin: the **nearest** is kept.
- No timestamps in v1 — the phone can only stamp arrival time (hence LDS2).
- Intensity is decoded by the firmware but **dropped** in v1 (hence LDS2).

### 2.2 HBT1 — telemetry, ESP32 → phone, UDP 9902, 50 Hz (`HB_TELEM_PERIOD_MS=20`), 78 B

Source of truth: `hb_link.h` sender + `hoverboard_udp_bridge.cpp` decoder.
The receiver accepts 42/44/52/74/78-byte historical lengths; the current
firmware always sends 78.

| Offset | Size | Field | Type | Unit / notes |
|--------|------|-------|------|--------------|
| 0 | 4 | magic | ASCII | `"HBT1"` |
| 4 | 2 | seq | u16 | telemetry sequence |
| 6 | 4 | millis | u32 | board `millis()` |
| 10 | 2 | flags | u16 | see below |
| 12 | 2 | fb_bytes | u16 | mainboard feedback length: 0=probing, 18/22/26 |
| 14 | 4 | ticks_left | i32 | cumulative wheel ticks, left |
| 18 | 4 | ticks_right | i32 | cumulative wheel ticks, right |
| 22 | 2 | rpm_left | i16 | measured RPM, + = forward |
| 24 | 2 | rpm_right | i16 | measured RPM, + = forward |
| 26 | 2 | battery | i16 | centi-volts |
| 28 | 2 | board_temp | i16 | deci-°C |
| 30 | 2 | curr_left | i16 | centi-amps |
| 32 | 2 | curr_right | i16 | centi-amps |
| 34 | 4 | frame_count | u32 | mainboard feedback frames |
| 38 | 4 | checksum_err | u32 | mainboard CRC errors |
| 42 | 2 | sonar_mm | u16 | FRONT sonar, mm; 0 = no echo |
| 44 | 2 | sent_steer | i16 | last steer sent to mainboard |
| 46 | 2 | sent_speed | i16 | last speed sent to mainboard |
| 48 | 2 | sonar_left_mm | u16 | 0 = no echo |
| 50 | 2 | sonar_right_mm | u16 | 0 = no echo |
| 52 | 2 | imu_status | u16 | see below |
| 54 | 8 | quat w,x,y,z | i16 ×4 | scale 1/16384 |
| 62 | 6 | gyro x,y,z | i16 ×3 | scale 1/16 deg/s |
| 68 | 6 | accel x,y,z | i16 ×3 | scale 1/100 m/s², gravity included |
| 74 | 1 | bat_state | u8 | 0 unknown, 1 ok, 2 low (speed ×0.5), 3 critical (stopped) |
| 75 | 1 | bat_cells | u8 | 10 |
| 76 | 2 | bat_filtered | u16 | centi-volts, low-passed |

`flags` bits (u16):

| Bit | Mask | Meaning |
|-----|------|---------|
| 0 | 0x0001 | mainboard up |
| 1 | 0x0002 | command timed out (wheels held 0) |
| 2 | 0x0004 | current feedback valid |
| 3 | 0x0008 | sonar obstacle (front) |
| 4 | 0x0010 | sonar back-off active |
| 5 | 0x0020 | front sonar stale |
| 6 | 0x0040 | ticks synthesised from RPM (18-byte upstream firmware) |
| 7 | 0x0080 | buzzer on |
| 8 | 0x0100 | left sonar obstacle |
| 9 | 0x0200 | right sonar obstacle |
| 10 | 0x0400 | sonar avoidance turn active |
| 11 | 0x0800 | left sonar stale |
| 12 | 0x1000 | right sonar stale |
| 13 | 0x2000 | left LED lit |
| 14 | 0x4000 | right LED lit |
| 15 | 0x8000 | back LED lit |

`imu_status` bits: 0 = IMU found, 1 = sample fresh, 2–3/4–5/6–7/8–9 =
calibration 0..3 of sys/gyro/accel/mag, 10 = compass (never set — the
magnetometer is not used).

**Ticks are running totals**, never deltas: a dropped datagram costs nothing;
the receiver re-baselines when `millis` goes backwards (board reboot).

### 2.3 HBC1 — wheel command, phone → ESP32, UDP 9901, 12 B (app sends at 20 Hz)

Source of truth: `hb_link.h` receiver.

| Offset | Size | Field | Type | Notes |
|--------|------|-------|------|-------|
| 0 | 4 | magic | ASCII | `"HBC1"` |
| 4 | 2 | seq | u16 | command sequence |
| 6 | 2 | rpm_left | i16 | + = forward |
| 8 | 2 | rpm_right | i16 | + = forward |
| 10 | 2 | flags | u16 | bit0 enable, bit1 buzzer |

- `bit0 clear` is an explicit stop, **not** silence: it still refreshes the
  command timeout. Silence (no datagram) for `HB_CMD_TIMEOUT_MS` stops the
  wheels (300 ms in the new firmware).
- The board acts on the **newest** valid datagram in the queue only.

---

## 3. Protocol v2 (phone brain)

Adds: single-owner claim with lease, NTP-style clock sync, command echo for
latency, lidar intensity + scan timestamps for de-skewing and dock
detection. The board **keeps decoding HBC1** from the owner for one
transition period.

### 3.1 HELO — claim/discovery, phone → board, UDP 9901

| Offset | Size | Field | Type | Notes |
|--------|------|-------|------|-------|
| 0 | 4 | magic | ASCII | `"HELO"` |
| 4 | 2 | proto | u16 | 2 |
| 6 | 2 | scan_port | u16 | where the phone wants LDS2 |
| 8 | 2 | telem_port | u16 | where the phone wants HBT2/PNG2 |
| 10 | 2 | flags | u16 | bit0 = takeover |
| 12 | 4 | session | u32 | random per app run, echoed |

### 3.2 HACK — claim reply, board → phone, to the HELO source

| Offset | Size | Field | Type | Notes |
|--------|------|-------|------|-------|
| 0 | 4 | magic | ASCII | `"HACK"` |
| 4 | 2 | proto | u16 | board's protocol version |
| 6 | 1 | status | u8 | 0 granted, 1 denied, 2 version_mismatch |
| 7 | 1 | board_type | u8 | 0 = ESP32-WROOM, 1 = ESP32-S3 |
| 8 | 2 | scan_bins | u16 | bins per LDS2 scan |
| 10 | 2 | ticks_per_rev | u16 | 90 |
| 12 | 4 | session | u32 | echo of HELO.session |
| 16 | 4 | board_ms | u32 | board `millis()` at reply |
| 20 | n | fw_version | ASCII | NUL-terminated, ≤ 32 B |

A second phone is **denied** unless it sets `flags bit0 (takeover)`, which
steals ownership. Why: two phones driving one robot is dangerous.

### 3.3 Lease

- Owner forgotten after **3 s** without any datagram from it.
- Phone sends PNG1 at ≥ 2 Hz while idle (they are the keepalive).
- With no owner the board sends nothing (no scans, no telemetry).

### 3.4 HBC2 — wheel command v2, phone → board, ~20 Hz, 16 B

| Offset | Size | Field | Type | Notes |
|--------|------|-------|------|-------|
| 0 | 4 | magic | ASCII | `"HBC2"` |
| 4 | 2 | seq | u16 | |
| 6 | 2 | rpm_l×10 | i16 | 0.1 rpm resolution |
| 8 | 2 | rpm_r×10 | i16 | 0.1 rpm resolution |
| 10 | 2 | flags | u16 | see below |
| 12 | 4 | phone_ms | u32 | phone monotonic ms (echoed in HBT2) |

`flags` bits: 0 enable, 1 buzzer, 2 **E-stop (latched)**, 3 **clear E-stop**.
Accepted from the owner only. Legacy HBC1 is still accepted from the owner.

### 3.5 PNG1 / PNG2 — clock sync + keepalive

PNG1 (phone → board, ≥ 2 Hz, 12 B):

| Offset | Size | Field | Type |
|--------|------|-------|------|
| 0 | 4 | magic | ASCII `"PNG1"` |
| 4 | 4 | seq | u32 |
| 8 | 4 | phone_ms | u32 |

PNG2 (board → phone, immediately, 20 B):

| Offset | Size | Field | Type |
|--------|------|-------|------|
| 0 | 4 | magic | ASCII `"PNG2"` |
| 4 | 4 | seq | u32 | echo |
| 8 | 4 | phone_ms | u32 | echo |
| 12 | 4 | esp_ms | u32 | board `millis()` |
| 16 | 4 | esp_us | u32 | board micros() within the ms (finer RTT) |

Phone computes NTP-style offset/RTT per sample and keeps the sample with the
**lowest RTT** (Wi-Fi jitter makes the running minimum of `phone_rx − esp_ts`
the robust estimator).

### 3.6 HBT2 — telemetry v2, board → owner's telem_port, 50 Hz, 96 B

Bytes 0–77 **identical to HBT1** except magic `"HBT2"`. Then:

| Offset | Size | Field | Type | Notes |
|--------|------|-------|------|-------|
| 78 | 2 | last_cmd_seq | u16 | last HBC2 seq accepted |
| 80 | 4 | last_cmd_phone_ms | u32 | echo of that command's phone_ms |
| 84 | 2 | cmd_age_ms | u16 | age of the last accepted command |
| 86 | 1 | link_flags | u8 | bit0 E-stop latched, bit1 web stick owns, bit2 OTA active, bit3 AP mode, bit4 STA connected |
| 87 | 1 | rssi | i8 | dBm |
| 88 | 1 | sonar_reflex | u8 | 0 none, 1 stop, 2 backoff, 3 turn |
| 89 | 1 | lidar_hz_deci | u8 | rotation rate, 0.1 Hz units |
| 90 | 2 | out_rpm_l×10 | i16 | wheel targets **after** reflex + battery scaling |
| 92 | 2 | out_rpm_r×10 | i16 | |
| 94 | 2 | lidar_crc_err | u16 | lidar CRC error count |
| 96 | — | total | | |

Why the echo fields: `phone_ms` round-tripped through HBT2 gives one-way
latency without a second clock domain. `out_rpm` after reflex shows the
truth of what the wheels were told, vs. HBC2's request.

### 3.7 LDS2 — scan v2, board → owner's scan_port, one datagram per revolution

Header **26 B**, then ranges, then intensity. 360 bins → 26 + 720 + 360 =
**1106 B** (fits one MTU).

| Offset | Size | Field | Type | Notes |
|--------|------|-------|------|-------|
| 0 | 4 | magic | ASCII | `"LDS2"` |
| 4 | 2 | bins | u16 | 360 default |
| 6 | 4 | scan_id | u32 | |
| 10 | 4 | start_ms | u32 | board ms when the revolution began |
| 14 | 4 | end_ms | u32 | board ms when it completed |
| 18 | 2 | rotation | u16 | centi-Hz (e.g. 5.5 Hz → 550) |
| 20 | 2 | points | u16 | raw points seen |
| 22 | 2 | valid | u16 | points with in-range distance |
| 24 | 2 | flags | u16 | bit0 = intensity section present |
| 26 | 2·bins | ranges | u16 LE | mm, 0 = no return |
| 26+2·bins | bins | intensity | u8 | 0..255 per bin |

Semantics (as v1): bin `i` is clockwise from the lidar zero mark; never
reordered on the board — the phone applies mirror + mounting yaw.
**Why start/end ms:** the phone de-skews the scan with its odometry/EKF
motion during the revolution. **Why intensity:** a reflective strip on the
dock appears as bright bins → dock detection.

### 3.8 BYE1 — release, board → owner (also sent when the owner changes)

| Offset | Size | Field | Type |
|--------|------|-------|------|
| 0 | 4 | magic | ASCII `"BYE1"` |
| 4 | 1 | reason | u8 | 0 released by owner, 1 lease expired, 2 takeover |

Wheels go to zero on release.

---

## 4. Decoder requirements (app side)

- Kotlin decoders in the app + unit tests **built from captured packets**
  (see `tools/phone_sim.py` transcripts and real captures under
  `android/VacuumNav/app/src/test/resources/captures/`).
- Tolerate v1 lengths 42/44/52/74/78 for HBT1 (older firmware).
- Unknown magic or short datagram → drop + count; never crash.
- The v1 bridge's historical quirk (`flip_lr`, `angle0_deg=180`) is **not**
  applied in the app decoders; the app owns mirroring/mounting via
  `RobotConfig.lidarMirror / lidarMountYawDeg` so the raw convention (clockwise
  from lidar zero) survives to the SLAM layer.

## 5. Reference packet sizes

| Packet | Direction | Size (360 bins) |
|--------|-----------|-----------------|
| LDS1 | board → phone | 734 B |
| LDS2 | board → phone | 1106 B |
| HBT1 | board → phone | 78 B |
| HBT2 | board → phone | 96 B |
| HBC1 | phone → board | 12 B |
| HBC2 | phone → board | 16 B |
| HELO | phone → board | 16 B |
| HACK | board → phone | 20 + n B |
| PNG1 | phone → board | 12 B |
| PNG2 | board → phone | 20 B |
| BYE1 | board → phone | 5 B |
