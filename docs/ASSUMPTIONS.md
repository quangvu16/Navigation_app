# ASSUMPTIONS — decisions, repo facts, and where the repo overrides the prompt

Rule applied throughout: **where the reference repo differs from the build
prompt, the repo wins**; each such case is listed below. Everything else is a
design decision of the new app, commented in code ("why") and in
`docs/ARCHITECTURE.md` / `docs/TUNING.md`.

## 0. Where things live

| Thing | Path |
|-------|------|
| This project (deliverables) | `/Volumes/0775999222/App/Navigation APP/` — `vacuum_core/`, `android/VacuumNav/`, `firmware/vacuum_phone_esp32/`, `docs/`, `tools/` |
| Reference repo (read-only, the "repo" below) | `/Volumes/0775999222/App/robot_ws/` — the `quangvu16/hoverboard-robot` ROS 2 workspace. **Never modified**; the three existing firmware sketches stay untouched there. |

The prompt listed repo paths relative to the repo root (`firmware/…`,
`src/esp_tcp_server/…`); the working directory was empty, so the repo is the
`robot_ws` checkout on the same volume (its layout matches the prompt
exactly: `firmware/ldrobot_ld14p_esp32_mpu9250_ros`, `src/esp_tcp_server`,
`src/hoverboard_base_node`, `src/robot_bringup`, `src/robot_navigation`,
`tools/calib_wheel_separation.py`, `android_app/`). New code goes in the
working directory; the new firmware sketch `firmware/vacuum_phone_esp32/` is
created **here**, which trivially satisfies "leave the three existing
sketches untouched".

## 1. Verified wire protocol (v1)

Byte layouts of LDS1, HBT1, HBC1 confirmed **from both sides**: firmware
(`lidar_ld14.h`, `hb_link.h`) and the reference decoders
(`src/esp_tcp_server/src/udp_lidar_receiver.cpp`,
`hoverboard_udp_bridge.cpp`). Full tables in `docs/PROTOCOL.md`. Facts that
matter downstream:

- LDS1 = 14 B header + 2·bins: `"LDS1"`, bins u16, scan_id u32, 4 B reserved
  zero, ranges mm u16. 360 bins → 734 B. **Bin i is clockwise from the
  lidar's zero mark**; never reordered on the board.
- HBT1 = 78 B at 50 Hz (`HB_TELEM_PERIOD_MS=20`); ticks are cumulative i32 at
  offset 14/18; receiver historically accepted 42/44/52/74/78 B lengths.
- HBC1 = 12 B; bit0 clear = explicit stop that still refreshes the timeout.
- Everything little-endian.

## 2. Robot constants — adopted from the repo (source of truth)

From `src/hoverboard_base_node/config/hoverboard_base.yaml`,
`firmware/…/config.h`, `hoverboard.h`:

| Constant | Value | Source | Notes |
|----------|-------|--------|-------|
| wheel_radius | **0.09615 m** | hoverboard_base.yaml (`measured 2026-09-26`, "19.23 cm wheel") | prompt agrees (96.15 mm). 2πr/90 = 6.71 mm/tick ✓ |
| track width (wheel_separation) | **0.18 m** | hoverboard_base.yaml, comment "**NOT yet verified**" | prompt agrees ("unverified → must calibrate"). Ships in RobotConfig, calibration wizard (Phase 2) must fix it |
| ticks/rev | **90** | `HB_TICKS_PER_REV=90` = `encoder_tpr` (15 pole pairs × 6 hall states) | encoder wrap 9000 (= 100 turns) already unwrapped by firmware |
| max wheel speed | 0.5 m/s (repo) / app default **0.35 m/s** | `max_wheel_speed_mps` vs prompt | app limit is a policy default in RobotConfig, editable in Settings; never above the repo's 0.5 |
| max angular speed | app default **1.2 rad/s** | prompt | Nav2 tuned at 1.0; both kept in TUNING.md |
| battery | 10S (36 V): low < 35.5 V (speed ×0.5), crit < 34.5 V (stop), recover > 37.0 V | `battery.h` / prompt agree | charge detection = `bat_filtered` voltage rise (no charge bit exists in HBT1; see §4) |
| command timeout | firmware **300 ms** (new), repo 1000 ms (`HB_CMD_TIMEOUT_MS`), mainboard 0.8 s, `cmd_timeout_s: 0.5` in yaml | prompt §4.4 | 300 ms < 11 cm coasting at 0.35 m/s |
| lidar | LD14 profile at **115200 baud**, `LIDAR_MEASURE_FREQ=2300`, auto-probe {115200, 230400, 460800, 921600} | config.h | unit sold as LD14P (230400) but answers at 115200 — keep auto-probe, report real baud/rotation in HACK/HBT2 (prompt agrees) |
| lidar bins | 360 (1° grid), ≤ 720 | `SCAN_BINS` | 360 = what ROS tooling expects; honest gaps over discarding points |
| lidar range | 0.02–8.0 m clipped on board; receiver advertised 0.02–20.0 | config.h / `ld14p.launch.py` | phone-side pre-filter also drops < 0.12 m (robot body) |
| lidar ranging-centre correction | OFFSET_X=5.9 mm, OFFSET_Y=−18.975571 mm, applied **in firmware** | `lidar_ld14.h` (LDROBOT SDK `sl_transform.cpp`) | do not re-apply in the app |
| rotation rate | ~5–12 Hz observed (11.96 Hz and 4.83 Hz in one unit's two runs) | config.h comment | LDS2 reports the real rate; deskew uses it |
| sonar | 3× HC-SR04: front stop < 350 / clear > 500 mm; sides stop < 50 / clear > 100 mm; 30 ms ping slots (front,left,front,right) | config.h / `sonar.h` | hysteresis 2 near / 3 far samples; stale after 5 s without a real echo |
| gyro | MPU9250 ±500 °/s (1/16 deg/s in HBT1), accel ±2 g (1/100 m/s²), 100 Hz sampled at 50 Hz, Mahony KP=1.0, bias cal while still 2 s | `mpu9250.h` | **no magnetometer** (hub-motor magnets) |
| IMU axes | chip axes; repo maps them via URDF `imu_joint` | hb_link.h comment | RobotConfig carries an IMU→base rotation for the same job |
| board feedback | 18/22/26-byte probed layouts, 115200 baud USART2, cmd 8 B every 50 ms, XOR checksum, counters mod 9000 | `hoverboard.h` | kept verbatim in new firmware |
| web stick | `http://hoverboard.local/`, WS `/ws`, owns wheels 500 ms | `web_drive.h` | kept; new link flags report "web stick owns" in HBT2 |

## 3. Tuning values adopted as hints (repo → new stack)

The prompt says Nav2/EKF params are **tuning hints only** — no ROS code is
ported. The repo has two Nav2 param sets; the one actually used for the real
robot is `src/robot_navigation/param/hoverboard.yaml` (MPPI), launched by
`navigation2.launch.py`; `config/nav2_params.yaml` is a stale TB3/RPP sim set
— **ignored** (recorded because mixing them up is easy).

| Hint | Value (hoverboard.yaml / ekf_hoverboard.yaml) | Use in vacuum_core |
|------|-----------------------------------------------|--------------------|
| costmap resolution | 0.02 m (local), 0.02 global | we use 0.05 m cells for SLAM grid (prompt), costmap 0.05; 0.02 kept as a tunable |
| footprint | square ±0.165 m, padding 0.01 | RobotConfig.robotRadius default **0.23 m** (half-diagonal of the footprint) + 0.10 m soft margin |
| inflation | 0.35 m | inflation radius = robotRadius + 0.10 soft margin (prompt) |
| max velocities | vx 0.25 m/s, wz 1.0 rad/s; accel 0.5 / 1.5 | app defaults 0.35 m/s / 1.2 rad/s (prompt §5), Settings-editable |
| controller | MPPI on the robot; RPP+RotationShim in the stale set | prompt mandates Regulated Pure Pursuit → we implement RPP (documented choice) |
| planner | Navfn, tolerance 0.3, allow_unknown | A* 8-connected (prompt §5) |
| goal tolerances | xy 0.20 m, yaw 0.25 rad | same defaults in RobotConfig |
| recovery | spin/backup/drive_on_heading/wait, rot 0.3–0.8 rad/s | stop → back up 0.15 m → rotate 90° → replan (prompt §5) |
| EKF (robot_localization) | 30 Hz, 2D mode, fuses odom **vx + vyaw** and IMU **vyaw only**; yaw orientation deliberately excluded (drifts) | custom 2D EKF [x,y,θ,v,ω]: predict from wheels, update ω from gyro Z — same fusion philosophy |
| obstacle raytrace | min 0.30 m (ignores chassis returns) | pre-filter drops < 0.12 m; body-angle mask configurable |
| AMCL alphas | 0.05 differential motion model | MCL odom noise defaults seeded from this |

## 4. Discrepancies and resolutions (prompt vs repo)

1. **Docking approach.** Prompt: go 0.5 m in front → lidar line-fit align →
   slow **forward** final approach until charge appears. Repo
   (`robot_docking/dock_manager.py`): Nav2 to a staging pose 0.7 m in front,
   then **reverse** in (front sonar reflex < 350 mm and collision monitor
   only watch the front; a forward final approach would be blocked by the
   very reflex that protects the robot). Resolution: keep the prompt's
   sequence but make the final leg **configurable** (`dockReverseIn`,
   default true = repo behaviour) because the front sonar reflex latches at
   350 mm and would otherwise freeze the last 0.35 m of a forward approach.
   Dock pose is a saved map pose ("dock is here") as in the repo.
2. **Charge detection.** Prompt says "charging flag … in HBT". No charge flag
   exists in HBT1 (or HBT2). Resolution: detect charge by `bat_filtered`
   voltage rise above the docked open-circuit level (~+0.5 V step) and/or
   `curr_*` sign if the mainboard reports it; reserve one `link_flags` bit in
   HBT2's next revision for a real charge input (Phase 8, needs hardware).
3. **HBT1 "78 B" vs bridge "42–78 B".** The decoder accepts 5 historical
   lengths. The app decoder will too (42/44/52/74/78), publishing only the
   field groups present — same policy as `hoverboard_udp_bridge.cpp`.
4. **`DEST_IP "192.168.1.12"` is compile-time in v1.** The phone replaces the
   PC, so v2 uses owner claim (HELO/HACK) instead; the constant disappears
   from the new firmware (`config.h` rewrite, prompt §4.1).
5. **`flip_lr=true, angle0_deg=180`** (v1 receiver node params) are the
   *ROS-side* correction of the clockwise bin convention. The app never bakes
   this into the decoder: `RobotConfig.lidarMirror` + `lidarMountYawDeg`
   own it (prompt §5 frames). The defaults to start from are mirror=true,
   yaw=180° — recorded so the first real-robot run matches the ROS setup.
6. **Two Nav2 param sets** (MPPI vs stale RPP): see §3 — only
   `hoverboard.yaml` values are used as hints.
7. **`behavior_server` uses `robot_base_frame: base_link`** while everything
   else uses `base_footprint`. Our frame chain is map→odom→base→laser;
   `base` ≙ `base_footprint` (robot centre at ground projection).
8. **Wheel radius looks "big" (96.15 mm ≈ 7.6" effective)** vs the "6.5 inch"
   hoverboard wheels (82.5 mm radius). The yaml says *measured* 2026-09-26
   (19.23 cm wheel) and the prompt adopts it; with tire compression an
   *effective* rolling radius above the unloaded geometry is normal. Trust
   the measured value; the Phase-2 calibration wizard re-checks it against
   tape-measured distance.
9. **Old `android_app/` (com.hoverboard.robotnav)** drives the robot via
   rosbridge v2 (ws://:9090) and depends on ROS nodes. It is **not reused**;
   the new `android/VacuumNav` talks to the ESP32 directly (no ROS).
10. **`calib_wheel_separation.py` docstring is stale** (points at
    `robot_bringup.launch.py` / `robot_esp.urdf`; the value lives in
    `hoverboard_base.yaml`). Our calibration wizard compares gyro yaw vs
    tick-derived yaw over N in-place turns (prompt §6) and writes
    RobotConfig.trackWidth.
11. **Command timeout**: yaml `cmd_timeout_s: 0.5`, firmware 1000 ms,
    mainboard 0.8 s. The new firmware uses **300 ms** (prompt §4.4) — below
    all of them; `WEB_DRIVE_TIMEOUT_MS` goes 500 → 250 ms to keep its
    `static_assert` ordering.

## 5. Explicitly unverified (open items, tracked)

- **Track width 0.18 m** — repo itself says unverified. Must calibrate
  before trusting odometry/SLAM scale in the rotation axis.
- **Lidar mounting offset + mirroring on the physical robot** — defaults
  mirror=true / 180° from the ROS setup; Diagnostics ships the mirror test
  (port of `tools/lidar_mirror_test.py`).
- **IMU mounting rotation** — chip axes (per mpu9250.h). RobotConfig default
  = identity + the known chip orientation; verify in Phase 2 (drive a square).
- **Whether this unit is an LD14 or LD14P** — config profile says LD14
  (115200/2300 Hz). Auto-probe keeps both working; HACK reports reality.
- **Real rotation speed stability** — two observed regimes (≈12 Hz and
  ≈4.8 Hz). Deskew and scan age thresholds must not assume 6 Hz.

## 6. Sensor-timing and clock assumptions

- The phone stamps every received datagram with
  `SystemClock.elapsedRealtimeNanos()` at arrival. If the board's `millis()`
  is present (HBT1/HBT2/LDS2), the offset is estimated NTP-style over
  PNG1/PNG2 keeping the lowest-RTT sample (Wi-Fi jitter dominates).
- EKF and scan de-skew run on **board time translated to phone time** via
  that offset; LDS2's start/end ms exist exactly for de-skewing (LDS1 has no
  timestamps — v1 scans can only be stamped at arrival and are deskewed with
  the *previous* motion estimate; acceptable, documented).
- Sim runs on a virtual monotonic clock it owns (no wall-clock coupling).

## 7. Software-stack assumptions (prompt §3 kept as-is)

- Kotlin + Jetpack Compose UI; all robotics math in C++17 `vacuum_core`
  (NDK + JNI; Eigen header-only; no OpenCV — erode/dilate/distance-transform/
  connected-components written in-library).
- Robot brain in a foreground service
  (`foregroundServiceType="connectedDevice"`) + partial WakeLock +
  WifiLock (LOW_LATENCY on API 29+, HIGH_PERF below).
- ESP32 keeps all safety (watchdog 300 ms, E-stop latch, sonar reflex,
  battery states). The app adds zero-RPM-on-every-stop-path as defense in
  depth.
- Map model follows Valetudo layer/entity naming (floor/wall/segment layers;
  robot/charger/path/virtual_wall/no_go_area entities) so export to Home
  Assistant is possible later.
- Phone placement (b) — phone mounted on the robot on the board's AP — is
  the recommended mode; (a) home Wi-Fi works with the same code.
- minSdk 26, Hilt, coroutines/Flow, kotlinx.serialization, Room (history),
  DataStore (settings).

## 8. Phase-1 scope decisions (this delivery)

- The simulator lives in `vacuum_core/sim/` per the prompt. PNG floor plans
  are decoded on the Kotlin side (android.graphics.Bitmap) and handed to the
  core as a grid; on the host, `SimWorld::loadPgm()` + synthetic grids keep
  the core dependency-free (no image library in C++).
- `SimRobotLink` implements the same `RobotLink` interface the UDP link
  will, producing HBT-shaped telemetry and LDS-shaped scans from the sim, so
  every later phase runs identically on emulator or hardware.
- Phase-1 acceptance: **the app shows a simulated robot driven by joystick
  at 60 fps**, with live lidar rendering; odometry/EKF are exercised by
  host unit tests (full app-visible EKF state comes with Phase 2's link).

## 9. Build-environment facts (this dev machine, 2026-11-19)

- **Xcode license not accepted** → the Apple compiler shims are gated. Fix
  without sudo: `export DEVELOPER_DIR=/Library/Developer/CommandLineTools`
  before CMake/Gradle builds. (Accepting the license with
  `sudo xcodebuild -license accept` would remove the need.)
- **AGP 9 has built-in Kotlin**: `org.jetbrains.kotlin.android` must NOT be
  applied (AGP 9 refuses it); the Compose compiler plugin
  (`org.jetbrains.kotlin.plugin.compose`) is still required separately.
  Stack: AGP 9.5.1 / Gradle 9.9.1 / Compose BOM 2026.09.00 / compileSdk 37
  (SDK platform `android-37.0`), JDK = Android Studio bundled JBR.
- **The project lives on an exFAT volume.** macOS stores xattrs (notably
  `com.apple.provenance`) as `._*` AppleDouble sidecar files on exFAT — one
  per created file/dir — which breaks AGP's directory-scanning tasks and
  CMake module globs. Mitigations in place:
  - `android/VacuumNav/settings.gradle.kts` redirects all build directories
    to `~/.cache/vacuum-nav/build/` when the root is on `/Volumes/`
    (override with `-Pvacuum.buildDir=...`);
  - `android/VacuumNav/.gradle` is a symlink to
    `~/.cache/vacuum-nav/gradle-project-cache`;
  - vendored toolchains in `.toolchain/` (CMake 3.31.6, Googletest 1.15.2,
    Gradle 9.9.1 — downloaded archives, extracted in place) had their
    sidecars purged after extraction (`find … -name "._*" -delete`);
  - git operations write pack files with provenance xattrs, so `._pack-*`
    sidecars appear inside `.git/objects/pack/` and make git print
    "non-monotonic index" errors — purge with
    `find .git -name "._*" -delete` before/after commits if they reappear.
- Tooling downloaded for the build lives in `.toolchain/` (not committed);
  the project's Gradle wrapper (committed) reproduces the build elsewhere.

## 10. Public-Wi-Fi provisioning + URL OTA (2026-11-20)

- **"Public WiFi" means an untrusted infrastructure LAN without client
  isolation** (home/office router). True hotspot networks (coffee-shop
  style AP isolation) block phone↔board UDP entirely — on those, use the
  BLE link, which needs no network at all; WFC1/WFO1 provisioning also
  works over BLE for exactly that reason. The board needs *outbound*
  internet only for WFO1's firmware download, never for the control link.
- **WFC1 trust model**: the owner lease (HELO claim) gates Wi-Fi changes,
  same as driving the robot — possession of the claimed link IS the
  credential, consistent with HBC2. A wrong password is recoverable: the
  board fails to join, STA_FALLBACK opens its own AP after 20 s, and you
  re-provision. `WFC1` with flags bit0 clears NVS back to `secrets.h`.
- **WFO1 trust model**: owner lease + `OTA_PASSWORD` on every request
  (ArduinoOTA parity). `https://` downloads use
  `NetworkClientSecure.setInsecure()` — encrypted but **not** CA-pinned;
  an active MITM on the LAN could substitute an image. On a network you
  don't control, prefer the LAN push path (`tools/ota_upload.sh`,
  ArduinoOTA is password-authenticated end-to-end), or serve the image
  from a host you trust. Documented here rather than silently accepted.
- **Flash budget**: 0.2.0 sits at 94–95 % of the default 1.3 MB dual-slot
  OTA partition scheme. The scheme must stay (two slots are what makes
  OTA safe); every future firmware feature has to budget from the
  remaining ~57–68 KB, and RAM headroom (~262 KB) is still comfortable.
- **NVS layout**: `Preferences` namespace `vacuum`, keys `ssid` (≤32 B)
  and `pass` (≤64 B, empty = open network). Absent keys = compile-time
  `secrets.h` pair; WFC1 flags bit0 deletes both keys.
