# ARCHITECTURE — why the pieces sit where they do

```
┌──────────────────────── Android phone ────────────────────────────┐
│ UI (Jetpack Compose)  ←→  ViewModels  ←→  RobotService            │
│                                           (foreground service)    │
│   RobotService owns:                                              │
│     Transport (RobotLink: SimRobotLink now, UdpRobotLink Phase 2) │
│     NativeEngine (C++ via JNI)  ── one thread, fixed-rate loop    │
│        Odometry+EKF → SLAM / Localization → Costmap               │
│        → Global planner (A*) → Local controller → wheel RPM       │
│        Coverage planner, Segmenter, Frontier explorer, Docking    │
│     MapStore (files per map), Scheduler, History                  │
└─────────────────────────────▲─────────────────────────────────────┘
                              │ UDP 9900/9901/9902 (Wi-Fi)
┌─────────────────────────────┴─────────────────────────────────────┐
│ ESP32: lidar UART, hoverboard UART, IMU, sonar, battery,          │
│ wheel PID, SAFETY (bumper/sonar stop, battery, command watchdog)  │
└───────────────────────────────────────────────────────────────────┘
```

## Key decisions (prompt §3, kept verbatim; see docs/ASSUMPTIONS.md for repo facts)

1. **Kotlin + Jetpack Compose UI; all robotics math in C++17 `vacuum_core`
   (NDK + JNI, Eigen header-only, no OpenCV).** SLAM and particle filters
   need tight loops and predictable memory; C++ is 5–20× faster than Kotlin
   for this and the same library can be wrapped for iOS later. The few image
   ops (erode/dilate, distance transform, connected components) are written
   in-library.
2. **The brain runs in an Android Foreground Service
   (`foregroundServiceType="connectedDevice"`)** with a partial WakeLock and
   a WifiManager.WifiLock (`WIFI_MODE_FULL_LOW_LATENCY` API 29+,
   `HIGH_PERF` below). Android kills/throttles background apps, and Wi-Fi
   power-save adds 100–300 ms latency that breaks a control loop. The UI can
   close; the service keeps cleaning. (Phase 1: service owns the simulator
   link, proving the ownership model before real hardware.)
3. **ESP32 keeps all safety.** The phone can freeze, ring, sleep or lose
   Wi-Fi; the board stops the wheels on its own (300 ms command watchdog,
   latched E-stop, sonar reflex, battery states — see firmware spec §4).
4. **One clock.** Every packet stamped with `SystemClock.elapsedRealtimeNanos()`
   on arrival; board time reconciled NTP-style over PNG1/PNG2 (lowest-RTT
   sample). EKF and scan matching go wrong with mis-aligned timestamps.
5. **Map model follows Valetudo** (layers floor/wall/segment; entities
   robot/charger/path/virtual_wall/no_go_area): proven vacuum feature set,
   easy export, publishable to Home Assistant later.
6. **Phone placement**: (a) phone off the robot on home Wi-Fi, or (b) phone
   mounted on the robot (board soft-AP, or USB-OTG serial later). (b) is the
   reliable mode; both run identical code.

## Frames (fixed)

```
map (world, metres, x right / y up) → odom → base (robot centre, x forward,
y left) → laser (offset + yaw + mirror flag from RobotConfig)
```

Screen rendering flips Y at draw time only. All goals, zones, walls and
poses travel through the stack in **world metres, never pixels**. The raw
lidar convention (bin i clockwise from the zero mark) survives to the scan
transform layer; see `vacuum_core/include/vacuum/scan_transform.h` — the one
tested place that turns bins into geometry (its Kotlin twin
`ui/ScanTransform.kt` feeds the renderer).

## Module map (Phase 1 delivered)

| Piece | Where | Notes |
|-------|-------|-------|
| Robot constants | `vacuum_core/include/vacuum/robot_config.h` + `robot/RobotConfig.kt` | one JSON, editable in Settings |
| Simulator | `vacuum_core/sim/` | diff-drive + slip + gyro noise + 6 Hz lidar (progressive beams → motion distortion for de-skew tests) + sonar |
| Link seam | `robot/RobotLink.kt` | SimRobotLink now; UdpRobotLink (protocol v2) Phase 2; UI/service cannot tell |
| Brain service | `service/RobotService.kt` | foreground + WakeLock + WifiLock; zero-RPM on every stop path |
| Map canvas | `ui/HomeScreen.kt` | 60 fps target, one Canvas node, lidar points + robot + sonar; pan/pinch |
| Host tests | `vacuum_core/tests/` (GoogleTest, 23 tests) + `app/src/test/` (JVM) | geometry, config JSON, raycast, dynamics, scan round trip |

## What each later phase adds (unchanged plan)

- Phase 2: `firmware/vacuum_phone_esp32` (protocol v2, docs/PROTOCOL.md),
  `UdpRobotLink`, odometry + EKF on device, calibration wizard.
- Phase 3: scan matcher + grid + pose graph + loop closure; MapStore.
- Phase 4: MCL localization, costmap, A*, regulated pure pursuit, recovery.
- Phase 5–8: zones/walls, frontier exploration, rooms + coverage, docking,
  schedule, multi-floor, polish.
