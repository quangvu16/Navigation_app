# vacuum_phone_esp32 — changelog

New sketch for the **phone brain** (no ROS, no PC). Derived from
`robot_ws/firmware/ldrobot_ld14p_esp32_mpu9250_ros` (kept untouched for the
ROS setup); the link layer was rewritten per prompt §4 and
`docs/PROTOCOL.md`.

## 0.1.0 — protocol v2 (this release)

Compiles clean on **both** FQBNs (arduino-cli, ESP32 core 4.0.0):

| FQBN | flash | RAM |
|------|-------|-----|
| `esp32:esp32:esp32` (WROOM) | 1 115 672 B (85 %) | 62 572 B (19 %) |
| `esp32:esp32:esp32s3` | 1 107 184 B (84 %) | 60 848 B (18 %) |

No third-party libraries: ESP32 core only (plus `tiny_hash.h`, our own
SHA-1/base64 for the WebSocket handshake).

### Rewritten

- **`phone_link.h`** (replaces `hb_link.h`): owner claim `HELO`/`HACK`
  (takeover flag steals a live owner), 3 s lease on any owner datagram
  (`PNG1` keepalive ≥ 2 Hz), `HBC2` commands at 20 Hz (legacy `HBC1` still
  accepted from the owner), `PNG1`→`PNG2` NTP-style clock sync (reply to
  the source port, `esp_us` for fine RTT), `HBT2` telemetry 96 B at 50 Hz
  (bytes 0–77 = HBT1 + command echo/latency, link flags, RSSI, reflex
  state, lidar rotation, post-reflex output RPM, lidar CRC errors), `BYE1`
  release (leased expired / takeover / released). No `DEST_IP` anywhere.
- **`config.h`**: one file for both boards (`#if CONFIG_IDF_TARGET_ESP32S3`
  pin map per `firmware/PINOUT.md`), `HB_CMD_TIMEOUT_MS` **1000 → 300 ms**
  (§4.4: < 11 cm coasting at 0.35 m/s), `WEB_DRIVE_TIMEOUT_MS` 500 → 250 ms
  (keeps `WEB < CMD` static_assert), `ROBOT_WIFI_MODE`
  STA_ONLY/AP_ONLY/**STA_FALLBACK** (default: AP after 20 s without a join),
  `netUp()` helper (STA associated **or** soft-AP up) replacing every
  `WiFi.status()==WL_CONNECTED` check, mDNS `_vacuum._udp` on 9901 with TXT
  `proto/board/fw`.
- **`lidar_ld14.h`**: decoder unchanged (CRC8, ranging-centre correction,
  baud auto-probe); sender is now **LDS2** — 26 B header (bins, scan_id,
  **start/end ms** for de-skew, centi-Hz rotation, points, valid, flags) +
  ranges + **intensity per bin** (v1 dropped it; the dock detector needs
  it), to the *owner's* scan port. Bins never reordered (clockwise from the
  zero mark — the phone applies mirror + mount yaw).
- **`sonar.h`**: new `SONAR_REFLEX_MODE` — **STOP (default)** strips only
  the forward component while any sonar is latched/stale (reverse and
  turn-in-place stay allowed: a blind back-off moves the robot behind the
  planner's back); `FULL` keeps the original back-off/turn-away reflex.
- **`web_drive.h`**: `netUp()`; SHA-1/base64 moved to `tiny_hash.h`
  (mbedtls's `sha1.h` moved under tf-psa-crypto in core 4.x and no longer
  resolves; our version is host-verified against RFC 6455 vectors).
- **`vacuum_phone_esp32.ino`**: no `DEST_IP`; three WiFi modes; mDNS
  `_vacuum` service; task layout unchanged (core 0: lidarTask +
  hoverboardTask@prio3; core 1: loop() + webTask) per §4.6.

### BLE transport (FriendBot-style)

- `ble_link.h`: GATT service/RX/TX with the FriendBot ESP32S3 UUIDs
  verbatim (UART-style service, write-without-response RX, notify TX +
  CCCD), MTU 517, advertising restart on disconnect. One GATT write = one
  datagram into the shared `phHandleDatagram()` — HELO claim, PNG1
  keepalive, HBC2, E-stop and the lease all work identically over BLE.
  TX framing: `[0x00]` whole datagram or `[0x01][idx][count]` LDS2 chunks
  (470 B). `phone_link.h` gained a `PhTarget` (UDP | BLE) reply abstraction
  used by every sender (HACK/PNG2/HBT2/LDS2/BYE1).
- Why: FriendBot's proven "phone drives the robot over BLE" path — no
  router, instant connect, keeps working when Wi-Fi does not. Both FQBNs
  still compile (WROOM 85 %, S3 84 % flash).

### Kept verbatim from the ROS sketch

`hoverboard.h` (FOC USART driver, PI loop, stall/wrong-way cutouts — only
the timeout constant changed via config), `mpu9250.h` (gyro/accel only, no
magnetometer), `battery.h`, `lights.h`, `ota.h`, `web_page.h`, pin-check
static_asserts.

### Bench tests (robot on blocks, §4.7 — need hardware)

- [ ] unplug Wi-Fi router → wheels stop < 300 ms
- [ ] E-stop latches against the web stick; survives lease expiry
- [ ] second phone denied (takeover flag steals)
- [ ] front sonar blocks forward but allows turn-in-place (STOP mode)
- [ ] LDS2 at the real rotation rate with intensity

Run them with `tools/phone_sim.py <board-ip>` (claim, drive, ping RTT,
E-stop, read-only monitor, `--capture FILE` for decoder test vectors).
