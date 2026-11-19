# VacuumNav — Android app (Kotlin + Jetpack Compose)

The phone-side robot brain: UI, `RobotService` (foreground service owning the
robot link), and `vacuum_core` (C++17, NDK) doing all robotics math.

## Layout

```
app/src/main/java/com/vacuum/nav/
  VacuumApp.kt          – Application; starts RobotService at process start
  robot/                – RobotConfig, RobotLink interface, SimRobotLink, NativeEngine (JNI)
  service/RobotService.kt – foreground service (connectedDevice) + WakeLock + WifiLock
  ui/                   – MainActivity, HomeScreen (60 fps map canvas), Joystick, ScanTransform, theme
app/src/main/cpp/       – jni_bridge.cpp + CMakeLists pulling in ../../vacuum_core
app/src/test/           – JVM unit tests (velocity->RPM, scan transform round trip)
```

## Build

Requirements: JDK 17+ (Android Studio JBR works), Android SDK with
`platforms;android-37`, NDK + CMake 3.22 (auto-fetched on first build).

```bash
cd android/VacuumNav
./gradlew :app:testDebugUnitTest   # host-side JVM tests
./gradlew :app:assembleDebug       # full build incl. vacuum_core via NDK
./gradlew :app:installDebug        # to a connected device (adb)
```

On this dev machine the Apple toolchain needs `DEVELOPER_DIR` pointed at the
CommandLineTools (the Xcode license is not accepted), and the JDK is the
Android Studio bundled one:

```bash
export JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home"
export DEVELOPER_DIR=/Library/Developer/CommandLineTools
```

`local.properties` carries `sdk.dir` (generated per machine).

**exFAT note:** this checkout sits on an exFAT volume where macOS writes
`._*` AppleDouble sidecar files that break AGP. `settings.gradle.kts`
relocates all build output to `~/.cache/vacuum-nav/build/` automatically in
that case, and `.gradle` is a symlink to `~/.cache/vacuum-nav/`.
See `docs/ASSUMPTIONS.md` §9.

## Phase 1 status

The app runs the **simulator** (`vacuum_core/sim`) through `SimRobotLink`,
driven by the on-screen joystick, rendering live lidar + robot pose at 60 fps.
Phase 2 swaps in `UdpRobotLink` (LDS2/HBT2/HBC2, docs/PROTOCOL.md) without UI
changes — `RobotLink` is the seam.

The C++ library is host-buildable and fully tested outside Android:

```bash
cmake -S ../../vacuum_core -B ../../vacuum_core/build -DVACUUM_BUILD_TESTS=ON
cmake --build ../../vacuum_core/build && ../../vacuum_core/build/vacuum_core_tests
```
