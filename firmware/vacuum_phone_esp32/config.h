// config.h - ALL set-up data for the phone-brain firmware, in one place:
// the pin map (ESP32-WROOM AND ESP32-S3), WiFi + the v2 link, the LiDAR
// model, the hoverboard mainboard mirror and speed loop, the HC-SR04 gate
// (with the new STOP reflex mode) and the buzzer.
//
// Differences from ldrobot_ld14p_esp32_mpu9250_ros/config.h (the ROS setup):
//   - NO DEST_IP: the phone claims ownership (HELO/HACK, phone_link.h) and
//     scans/telemetry go to the owner's announced ports.
//   - HB_CMD_TIMEOUT_MS 1000 -> 300 ms (prompt §4.4: < 11 cm of coasting at
//     0.35 m/s). WEB_DRIVE_TIMEOUT_MS 500 -> 250 so its static_assert
//     (web hand-back must beat the mainboard timeout) keeps holding.
//   - One config for both boards: pin map selected by CONFIG_IDF_TARGET_*.
//   - SONAR_REFLEX_MODE: STOP (new default, prompt §4.4) or FULL (the
//     original back-off / turn-away reflex).
// Robot geometry (wheel radius, track, speed limits) still lives on the
// phone in RobotConfig - never in here.
#pragma once

#include <Arduino.h>
#include <WiFi.h>

// -------------------------------------------------------------------- pin map
// Every GPIO this sketch touches. Both boards share the SAME wiring plan;
// only the numbering differs (firmware/PINOUT.md).
//
//   WROOM: LiDAR TX 16, buzzer 25, mainboard 26/27, LEDs 5/13/14,
//          sonar front 32/33, sides 17/19 + 18/23, I2C 21/22.
//   S3:    mainboard 6/7, buzzer 21, sonar front 1/2, ECHOs 10/11,
//          I2C 8/9; LiDAR 16, TRIGs 17/18, LEDs 5/13/14 unchanged.
//
// Grounds: LiDAR, mainboard, HC-SR04s and LEDs must all join ESP32 GND.
// Power the LiDAR (5 V, ~200 mA) and the HC-SR04s (5 V) from the robot's
// 5 V rail, not the DevKit's 5V pin while it runs on USB. Never connect
// the mainboard's VCC pin.
//
// Pins NOT to wire anything to: 6-11 (WROOM flash), 12 (strapping, flash
// voltage), 0 (bootloader), 1/3 (UART0), 34-39 (input only, no pull-down).
#if defined(CONFIG_IDF_TARGET_ESP32S3)
static const int LIDAR_RX_PIN   = 16;   // LiDAR TX -> here
static const int LIDAR_TX_PIN   = -1;
static const int BUZZER_PIN     = 21;
static const int HB_RX_PIN      = 6;    // mainboard TX -> this pin
static const int HB_TX_PIN      = 7;    // mainboard RX <- this pin
static const int SONAR_TRIG_PIN = 1;
static const int SONAR_ECHO_PIN = 2;
static const int SONAR_LEFT_TRIG_PIN  = 17;
static const int SONAR_LEFT_ECHO_PIN  = 10;
static const int SONAR_RIGHT_TRIG_PIN = 18;
static const int SONAR_RIGHT_ECHO_PIN = 11;
static const int LED_LEFT_PIN  = 14;
static const int LED_RIGHT_PIN = 13;
static const int LED_BACK_PIN  = 5;
static const int IMU_SDA_PIN   = 8;
static const int IMU_SCL_PIN   = 9;
#elif defined(CONFIG_IDF_TARGET_ESP32)
static const int LIDAR_RX_PIN   = 16;
static const int LIDAR_TX_PIN   = -1;
static const int BUZZER_PIN     = 25;
static const int HB_RX_PIN      = 26;
static const int HB_TX_PIN      = 27;
static const int SONAR_TRIG_PIN = 32;
static const int SONAR_ECHO_PIN = 33;
static const int SONAR_LEFT_TRIG_PIN  = 17;
static const int SONAR_LEFT_ECHO_PIN  = 19;
static const int SONAR_RIGHT_TRIG_PIN = 18;
static const int SONAR_RIGHT_ECHO_PIN = 23;
static const int LED_LEFT_PIN  = 14;
static const int LED_RIGHT_PIN = 13;
static const int LED_BACK_PIN  = 5;
static const int IMU_SDA_PIN   = 21;
static const int IMU_SCL_PIN   = 22;
#else
#error "vacuum_phone_esp32: build for esp32:esp32:esp32 or esp32:esp32:esp32s3"
#endif

// Compile-time pin checks (C++11 constexpr, so recursion instead of loops).
namespace pincheck {
constexpr bool usable(int p) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  // S3: flash/PSRAM 22-37, strapping 0/3/45/46, USB 19/20, UART0 43/44,
  // RGB 38/48 are unsafe; everything else that exists is fair game.
  // (17/18 are the side-sonar TRIG outputs and ARE usable - see PINOUT.md.)
  return p >= 0 && p <= 48 && !(p >= 22 && p <= 37) && p != 0 && p != 3 &&
         p != 45 && p != 46 && p != 19 && p != 20 && p != 43 && p != 44 &&
         p != 38 && p != 48;
#else
  return p >= 0 && p <= 39 && !(p >= 6 && p <= 11) && p != 12 && p != 0 &&
         p != 1 && p != 3 && p != 20 && p != 24 && !(p >= 28 && p <= 31);
#endif
}
constexpr bool output(int p) { return usable(p) && p < 34; }
constexpr int kAll[] = {LIDAR_RX_PIN, BUZZER_PIN,
                        HB_RX_PIN, HB_TX_PIN, SONAR_TRIG_PIN, SONAR_ECHO_PIN,
                        SONAR_LEFT_TRIG_PIN, SONAR_LEFT_ECHO_PIN,
                        SONAR_RIGHT_TRIG_PIN, SONAR_RIGHT_ECHO_PIN,
                        LED_LEFT_PIN, LED_RIGHT_PIN, LED_BACK_PIN,
                        IMU_SDA_PIN, IMU_SCL_PIN};
constexpr int kN = sizeof(kAll) / sizeof(kAll[0]);
constexpr bool unique_from(int i, int j) {
  return i >= kN ? true
       : j >= kN ? unique_from(i + 1, i + 2)
       : kAll[i] != kAll[j] && unique_from(i, j + 1);
}
}  // namespace pincheck
static_assert(pincheck::unique_from(0, 1), "two functions share one GPIO");
static_assert(pincheck::usable(LIDAR_RX_PIN) && pincheck::usable(HB_RX_PIN) &&
              pincheck::output(SONAR_ECHO_PIN) && pincheck::output(SONAR_LEFT_ECHO_PIN) &&
              pincheck::output(SONAR_RIGHT_ECHO_PIN),
              "input pin is flash/strapping/UART0/no-pull-down/nonexistent");
static_assert(LIDAR_TX_PIN == -1,
              "LIDAR_TX_PIN: add the pin to kAll and the output check");
static_assert(pincheck::output(BUZZER_PIN) && pincheck::output(HB_TX_PIN) &&
              pincheck::output(SONAR_TRIG_PIN) && pincheck::output(SONAR_LEFT_TRIG_PIN) &&
              pincheck::output(SONAR_RIGHT_TRIG_PIN) && pincheck::output(LED_LEFT_PIN) &&
              pincheck::output(LED_RIGHT_PIN) && pincheck::output(LED_BACK_PIN) &&
              pincheck::output(IMU_SDA_PIN) && pincheck::output(IMU_SCL_PIN),
              "output pin is flash/strapping/UART0/input-only/nonexistent");

// ------------------------------------------------------------- Serial Monitor
static const uint32_t SERIAL_MONITOR_BAUD = 115200;

// ----------------------------------------------------------------------- WiFi
// WIFI_SSID, WIFI_PASS and OTA_PASSWORD live in secrets.h (git-ignored).
// For the AP modes the SSID also doubles as the access point name.
#include "secrets.h"
#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif
// Runtime override (WFC1 provisioning, docs/PROTOCOL.md §3.9): NVS-backed
// SSID/pass chosen from the app; falls back to the secrets.h pair above.
#include "wifi_cfg.h"

#define ROBOT_HOSTNAME "hoverboard"

// ------------------------------------------------- WiFi mode (prompt §4.5)
// STA_ONLY   - home Wi-Fi only.
// AP_ONLY    - the board is the access point "VacuumBot" (best when the
//              phone rides on the robot).
// STA_FALLBACK (default) - join home Wi-Fi; if not joined within
//              WIFI_FALLBACK_MS, open the AP too (STA+AP).
#define WIFI_MODE_STA_ONLY 0
#define WIFI_MODE_AP_ONLY 1
#define WIFI_MODE_STA_FALLBACK 2
#ifndef ROBOT_WIFI_MODE
#define ROBOT_WIFI_MODE WIFI_MODE_STA_FALLBACK
#endif
static const uint32_t WIFI_FALLBACK_MS = 20000;  // AP opens if STA not joined
static const uint32_t WIFI_RETRY_MS    = 10000;
static const char *const AP_SSID = "VacuumBot";
// AP password: >= 8 chars or the soft-AP is open (fine on a lab bench).
static const char *const AP_PASS = "vacuumbot";

// -------------------------------------------------------- phone link (v2)
// phone_link.h implements docs/PROTOCOL.md. Owner claim + lease replace the
// old compile-time DEST_IP.
static const uint16_t PH_PROTO_VERSION = 2;
static const uint16_t PH_PORT_DEFAULT_CMD   = 9901;  // HELO/HBC2/PNG1 land here
static const uint16_t PH_PORT_DEFAULT_TELEM = 9902;  // HBT2/PNG2/BYE1
static const uint16_t PH_PORT_DEFAULT_SCAN  = 9900;  // LDS2
static const uint32_t PH_LEASE_TIMEOUT_MS   = 3000;  // owner forgotten (§4.3)
static const uint32_t PH_PNG_PERIOD_MS      = 500;   // phone keepalive >= 2 Hz
static const uint32_t PH_HELMET = 0;                 // reserved
static const uint32_t PH_TELEM_PERIOD_MS = 20;       // HBT2 at 50 Hz
// Resend the setpoint to the mainboard this often. Well inside its own
// 0.8 s timeout (hoverboard.h enforces this with its last_cmd_sent_ms_).
static const uint32_t HB_CMD_PERIOD_MS = 50;
// Stop the wheels if no command has been set for this long (prompt §4.4:
// 300 ms = < 11 cm coasting at 0.35 m/s; the mainboard itself stops at
// 0.8 s, the web stick hands back at 250 ms).
static const uint32_t HB_CMD_TIMEOUT_MS = 300;
static const uint32_t HB_FEEDBACK_TIMEOUT_MS = 500;
static const uint32_t HB_RESTART_GAP_MS = 200;
static const uint32_t HB_LOG_PERIOD_MS = 5000;
static const char *const FW_VERSION = "vacuum-phone-0.1.0";

// ---------------------------------------------------------------------- LiDAR
// UART1 on LIDAR_RX_PIN / LIDAR_TX_PIN (pin map above).
//
// Model profile. This unit answers at 115200, which is the LD14 profile; the
// LD14P uses 230400 and measures 4000 points/s. Auto-probe below keeps both
// working; HACK/HBT2 report the real baud and rotation rate.
static const uint32_t LIDAR_BAUD         = 115200;
static const uint32_t LIDAR_MEASURE_FREQ = 2300;
static const uint32_t CANDIDATE_BAUD[] = {115200, 230400, 460800, 921600};

// One bin per SCAN_BINS-th of a turn (1 deg grid = what tooling expects;
// discarding measurements is worse than leaving honest gaps). 360 x (2+1)
// bytes + 26 header = 1106 B for LDS2 - one MTU.
static const uint16_t SCAN_BINS = 360;
static const uint16_t RANGE_MIN_MM = 20;
static const uint16_t RANGE_MAX_MM = 8000;
static const uint32_t SCAN_MAX_AGE_MS = 500;

// ----------------------------------------------------------------- hoverboard
// Driver: hoverboard.h. Wire protocol constants stay there; everything below
// is set-up data. MUST match the MAINBOARD build (Inc/config.h it was
// flashed with).
static const float    HB_MB_SPEED_COEFF = 1.0f;
static const float    HB_MB_STEER_COEFF = 0.5f;
static const float    HB_MB_N_MOT_MAX = 1000.0f;
static const bool     HB_MB_INVERT_L_DIRECTION = false;
static const bool     HB_MB_INVERT_R_DIRECTION = false;
static const uint32_t HB_BAUD = 115200;
static const int32_t  HB_ENCODER_WRAP  = 9000;
static const int32_t  HB_TICKS_PER_REV = 90;

// How the mainboard sits in the chassis (robot on blocks to change).
static const bool HB_SWAP_WHEELS  = false;
static const bool HB_INVERT_LEFT  = false;
static const bool HB_INVERT_RIGHT = false;

// ESP32 speed loop (VLT_MODE mainboards; set HB_SPEED_LOOP_ON 0 for a
// fork in SPD_MODE where the board closes its own loop).
#ifndef HB_SPEED_LOOP_ON
#define HB_SPEED_LOOP_ON 1
#endif
static const bool     HB_SPEED_LOOP     = HB_SPEED_LOOP_ON;
static const float    HB_SL_STATIC      = 60.0f;
static const float    HB_SL_KFF         = 1.0f;
static const float    HB_SL_KP          = 2.0f;
static const float    HB_SL_KI          = 8.0f;
static const float    HB_SL_I_MAX       = 300.0f;
static const float    HB_SL_OVER_DECAY  = 0.85f;
static const float    HB_SL_BREAKAWAY_KEEP = 0.3f;
static const float    HB_SL_U_MAX       = 500.0f;
static const float    HB_SL_TARGET_MIN  = 0.5f;
static const uint32_t HB_SL_STALL_MS    = 1500;
static const float    HB_SL_STALL_RPM   = 3.0f;
static const uint32_t HB_SL_WRONGWAY_MS = 300;
static const float    HB_SL_WRONGWAY_RPM = 8.0f;

// -------------------------------------------------------------- HC-SR04 sonar
// Three sensors: front, left, right. One fires per slot in the order
// front, left, front, right so they never hear each other.
static const uint32_t SONAR_SLOT_MS     = 30;
static const uint32_t SONAR_ECHO_TIMEOUT_US = 25000;
static_assert(SONAR_SLOT_MS >= SONAR_ECHO_TIMEOUT_US / 1000 + 5,
              "a slot must outlast one echo timeout");
static const uint32_t SONAR_ECHO_MIN_US = 100;

// FRONT sensor hysteresis (reverse below STOP, resume above CLEAR).
static const uint16_t SONAR_STOP_MM  = 350;
static const uint16_t SONAR_CLEAR_MM = 500;
// SIDE sensors: turn away below SIDE_STOP_MM, resume above SIDE_CLEAR_MM.
static const uint16_t SONAR_SIDE_STOP_MM  = 50;
static const uint16_t SONAR_SIDE_CLEAR_MM = 100;
static const uint8_t SONAR_NEAR_SAMPLES = 2;
static const uint8_t SONAR_FAR_SAMPLES  = 3;

static const float    SONAR_BACKOFF_RPM    = 25.0f;
static const uint32_t SONAR_BACKOFF_MAX_MS = 1500;
static const float    SONAR_TURN_RPM    = 20.0f;
static const uint32_t SONAR_TURN_MAX_MS = 1500;
static const uint32_t SONAR_STALE_MS = 5000;
static const bool SONAR_REQUIRED = true;
static const bool SONAR_SIDES_REQUIRED = true;

// Reflex mode (prompt §4.4). STOP is the new default: only remove the
// FORWARD component when a sonar is latched or stale; reversing and
// turning in place stay allowed. A blind back-off moves the robot behind
// the planner's back. FULL = the original back-off / turn-away reflex.
#define SONAR_REFLEX_STOP 0
#define SONAR_REFLEX_FULL 1
#ifndef SONAR_REFLEX_MODE
#define SONAR_REFLEX_MODE SONAR_REFLEX_STOP
#endif

// --------------------------------------------------------------------- buzzer
static const bool BUZZER_ACTIVE_HIGH = true;

// --------------------------------------------------------------------- lights
static const bool     LIGHT_ACTIVE_HIGH   = true;
static const uint32_t LIGHT_BLINK_MS      = 300;
static const float    LIGHT_STOPPED_RPM   = 1.0f;
static const float    LIGHT_TURN_DIFF_RPM = 4.0f;

// ------------------------------------------------------------------ MPU9250
// Gyro + accel only: the hub motors' magnets corrupt the compass.
#define IMU_ENABLED 1
static const uint32_t IMU_I2C_HZ = 400000;
static const uint32_t IMU_RETRY_MS = 2000;

// ------------------------------------------------------------------- battery
// 10S pack thresholds (battery.h). Keep CRIT above the mainboard's
// BAT_DEAD (3.37 V/cell).
#ifndef BAT_PROTECT_ENABLED
#define BAT_PROTECT_ENABLED 1
#endif
#define BAT_BEEP_ENABLED 1
static constexpr float BAT_CELLS              = 10;
static constexpr float BAT_LOW_V_PER_CELL     = 3.55f;
static constexpr float BAT_CRIT_V_PER_CELL    = 3.45f;
static constexpr float BAT_RECOVER_V_PER_CELL = 3.70f;
static constexpr float BAT_LOW_SPEED_SCALE    = 0.5f;
static constexpr float BAT_FILTER_S           = 5.0f;
static const uint32_t BAT_CONFIRM_MS         = 10000;
static const uint32_t BAT_SAMPLE_MS          = 100;
static const uint32_t BAT_LOW_BEEP_MS        = 30000;
static const uint32_t BAT_CRIT_BEEP_MS       = 5000;
static_assert(BAT_CRIT_V_PER_CELL < BAT_LOW_V_PER_CELL && BAT_LOW_V_PER_CELL < BAT_RECOVER_V_PER_CELL,
              "battery thresholds must be CRIT < LOW < RECOVER");
static_assert(BAT_CRIT_V_PER_CELL > 3.37f, "CRIT must stay above BAT_DEAD");

// ------------------------------------------------------------- web drive
// The drive page shares the board with the phone link; either can own the
// wheels (web stick wins while held, HBT2 link_flags bit1 says so).
#ifndef WEB_DRIVE_ENABLED
#define WEB_DRIVE_ENABLED 1
#endif
static const uint16_t WEB_DRIVE_PORT = 80;
static const float    WEB_DRIVE_MAX_RPM = 60.0f;
// Hand-back must beat HB_CMD_TIMEOUT_MS (250 < 300).
static const uint32_t WEB_DRIVE_TIMEOUT_MS = 250;
static const uint32_t WEB_STATUS_PERIOD_MS = 200;
static_assert(WEB_DRIVE_TIMEOUT_MS < HB_CMD_TIMEOUT_MS, "web hand-back must beat the mainboard timeout");

// ------------------------------------------------------------------------ OTA
#ifndef OTA_ENABLED
#define OTA_ENABLED 1
#endif
static const uint16_t OTA_PORT = 3232;

// ------------------------------------------------- resolved at boot, not edited
// Set by the sketch's wifiBegin() when the soft-AP is up (AP_ONLY or the
// STA_FALLBACK fallback leg). netUp() below is the ONE "can this board send
// datagrams?" test (prompt §4.1): STA associated OR soft-AP running, so the
// link, the web page and OTA all work in AP mode where WiFi.status() never
// says WL_CONNECTED.
static bool wifi_ap_up = false;

static inline bool netUp() {
  if (wifi_ap_up) return true;
  return WiFi.status() == WL_CONNECTED;
}
