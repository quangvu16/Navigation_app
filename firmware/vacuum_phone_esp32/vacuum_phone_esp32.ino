// vacuum_phone_esp32 - the ESP32 for the PHONE-brain robot (no ROS, no PC).
//
//   LD14  --UART1--> ESP32 --"LDS2" UDP (owner's scan port)--> phone
//   mainboard <--UART2--> ESP32 <--"HBT2" 50 Hz / "HBC2" 20 Hz--> phone
//                          HELO/HACK claim, PNG1/PNG2 clock sync, BYE1
//   MPU9250 --I2C--> ESP32 (inside HBT2) -------------------------->  EKF
//   phone --http://<board>/ + ws://<board>/ws--> ESP32 (drive stick)
//
// This sketch REPLACES the ROS link (hb_link.h / HBT1 / HBC1 to a fixed
// DEST_IP) with protocol v2: the phone claims ownership, everything else is
// gated on that lease, and the board keeps ALL the safety (300 ms command
// watchdog, latched E-stop, sonar reflex, battery states) so it still works
// when the phone freezes, rings, sleeps or loses Wi-Fi (prompt §4).
//
// Byte tables: docs/PROTOCOL.md. Decisions: docs/ASSUMPTIONS.md.
//
//   config.h       ALL set-up data: both pin maps, WiFi modes, link,     <- the
//                  LiDAR model, mainboard mirror, speed loop,            only file
//                  sonars, lights, buzzer, battery                       you edit
//   phone_link.h   HELO/HACK, lease, PNG1/PNG2, HBC2/HBC1, HBT2, BYE1,
//                  the hoverboard task, reflex + E-stop
//   hoverboard.h   mainboard USART driver (unchanged, timeout 300 ms)
//   sonar.h        three HC-SR04s + reflex (STOP mode default, FULL optional)
//   lights.h       turn signals and stop light
//   lidar_ld14.h   LD14 decoder, scan binning, LDS2 sender (intensity!)
//   mpu9250.h      MPU9250 IMU (I2C raw + ESP32 fusion), in every HBT2
//   battery.h      low-battery protection: limit speed, then stop
//   web_drive.h    drive page + WebSocket server (no PC needed)
//   web_page.h     the page itself
//   ota.h          firmware updates over Wi-Fi (tools/ota_upload.sh)
//
// Task layout (prompt §4.6). Core 0: lidarTask (LiDAR UART) and
// hoverboardTask (mainboard UART, sonar, IMU, phone socket, watchdog,
// reflex), the latter at higher priority - a late scan costs one revolution,
// a late command frame lets the mainboard hit its own serial timeout. Core 1:
// loop() (Wi-Fi, OTA, scan send) and webTask. Owner IP/ports are behind a
// portMUX spinlock in phone_link.h (IPAddress is not atomic).
//
// No third-party libraries: only the ESP32 core (WiFi, Wire, ESPmDNS,
// mbedtls, ArduinoOTA).

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>

#include "config.h"
#include "phone_link.h"   // before lidar: the scan sender needs the owner
#include "ble_link.h"     // FriendBot-style BLE transport (optional)
#include "lidar_ld14.h"   // after phone_link: provides phLidarFreqHz/CrcErrors

// ----------------------------------------------------------------------- WiFi
// Nothing here blocks: WiFi.begin() is fired once and the status is polled,
// so a wrong SSID or a router still booting never wedges the sketch. Both
// tasks keep running throughout; datagrams are simply not sent until netUp()
// is true, and the hoverboard stops on its own command timeout meanwhile.
//
// ROBOT_WIFI_MODE (config.h):
//   STA_ONLY     - home Wi-Fi only.
//   AP_ONLY      - the board IS the access point "VacuumBot" (best when the
//                  phone rides on the robot).
//   STA_FALLBACK - home Wi-Fi first; if not joined within WIFI_FALLBACK_MS
//                  open the AP too (STA+AP). Default.
static bool     net_up_before = false;
static uint32_t wifi_retry_ms = 0;
static uint32_t sta_begin_ms = 0;

static void apStart() {
  if (wifi_ap_up) return;
  // Soft-AP alongside STA so a later STA join still works (AP_STA).
  if (WiFi.getMode() == WIFI_STA) WiFi.mode(WIFI_AP_STA);
  wifi_ap_up = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("WiFi AP \"%s\" up at %s\n", AP_SSID,
                WiFi.softAPIP().toString().c_str());
}

static void wifiBegin() {
  WiFi.disconnect(true);
  WiFi.setSleep(false);   // power save adds ~100 ms of latency to every packet
#if ROBOT_WIFI_MODE == WIFI_MODE_AP_ONLY
  WiFi.mode(WIFI_AP);
  apStart();
  Serial.println("WiFi mode: AP_ONLY (no home Wi-Fi)");
#else
  WiFi.mode(WIFI_STA);
  // Runtime credentials (wifi_cfg.h): NVS if the app provisioned a network,
  // else the compile-time secrets.h pair.
  WiFi.begin(wifiCfgSsid(), wifiCfgPass());
  sta_begin_ms = millis();
  Serial.printf("WiFi mode: %s, joining \"%s\"\n",
#if ROBOT_WIFI_MODE == WIFI_MODE_STA_FALLBACK
                "STA_FALLBACK (AP after " "20" " s if no join)",
#else
                "STA_ONLY",
#endif
                wifiCfgSsid());
#endif
  wifi_retry_ms = millis();
}

// One mDNS name for everything this board serves: the drive page (http), the
// vacuum link (_vacuum._udp on the command port, TXT proto/board/fw) and OTA.
static void mdnsUp() {
  if (!MDNS.begin(ROBOT_HOSTNAME)) {
    Serial.println("mDNS failed; use the IP address instead of " ROBOT_HOSTNAME ".local");
    return;
  }
#if WEB_DRIVE_ENABLED
  MDNS.addService("http", "tcp", WEB_DRIVE_PORT);
  Serial.printf("web: drive page on http://%s/ and http://%s.local/\n",
                WiFi.softAPIP().toString().c_str(), ROBOT_HOSTNAME);
#endif
  // _vacuum._udp on the command port with TXT proto/board/fw (prompt §4.5):
  // how the app discovers boards without a scan sweep.
  MDNS.addService("vacuum", "udp", PH_PORT_DEFAULT_CMD);
  MDNS.addServiceTxt("vacuum", "udp", "proto", "2");
  MDNS.addServiceTxt("vacuum", "udp", "board",
#if defined(CONFIG_IDF_TARGET_ESP32S3)
                     "esp32s3"
#else
                     "esp32"
#endif
  );
  MDNS.addServiceTxt("vacuum", "udp", "fw", FW_VERSION);
  if (otaEnabled()) MDNS.enableArduino(OTA_PORT, true);
}

static void wifiLoop() {
  // WFC1 provisioning (wifi_cfg.h): once the100 ms ack-grace elapsed,
  // switch the STA to the new network. The up/down edge logic below then
  // re-runs mDNS + the lidar sender hookup on the new link.
  if (wifiCfgApplyPending()) {
    Serial.printf("WiFi: switching to \"%s\"\n", wifiCfgSsid());
    WiFi.disconnect();
    WiFi.begin(wifiCfgSsid(), wifiCfgPass());
    sta_begin_ms = millis();
    wifi_retry_ms = millis();
    MDNS.end();            // force a clean mDNS re-announce on the new net
    net_up_before = false; // treat the next up as a fresh link-up edge
  }

  const bool sta_connected = (WiFi.status() == WL_CONNECTED);

#if ROBOT_WIFI_MODE == WIFI_MODE_STA_FALLBACK
  // Home Wi-Fi first; after WIFI_FALLBACK_MS without a join, open the AP too.
  if (!sta_connected && !wifi_ap_up &&
      millis() - sta_begin_ms >= WIFI_FALLBACK_MS) {
    Serial.println("WiFi: STA not joined in time, opening the AP as well");
    apStart();
  }
#endif

  // Retries: keep trying the STA (even while the AP is up) so a router that
  // was still booting at power-on does not strand the board.
  if (!sta_connected && millis() - wifi_retry_ms >= WIFI_RETRY_MS) {
    Serial.printf("WiFi \"%s\" still down, retrying\n", wifiCfgSsid());
    WiFi.begin(wifiCfgSsid(), wifiCfgPass());
    wifi_retry_ms = millis();
  }

  const bool up = netUp();
  if (up && !net_up_before) {
    Serial.printf("link up: STA %s  AP %s  IP %s\n",
                  sta_connected ? WiFi.localIP().toString().c_str() : "down",
                  wifi_ap_up ? WiFi.softAPIP().toString().c_str() : "off",
                  (sta_connected ? WiFi.localIP() : WiFi.softAPIP()).toString().c_str());
    lidarOnWifiUp();
    mdnsUp();
    otaBegin();   // once; later reconnects keep the same listener
  } else if (!up && net_up_before) {
    Serial.println("link down");
    lidarOnWifiDown();
    MDNS.end();
  }
  net_up_before = up;
}

// ---------------------------------------------------------------------- setup
void setup() {
  Serial.begin(SERIAL_MONITOR_BAUD);
  delay(200);
  Serial.printf("\nvacuum_phone_esp32 %s - phone brain (protocol v2), no ROS\n",
                FW_VERSION);

  lidarBegin();
  phBegin();      // starts sending zero commands at once, so the mainboard arms
  phBleBegin();   // FriendBot-style BLE transport alongside UDP

  // Core 0: the two real-time tasks (hoverboard higher priority, §4.6).
  xTaskCreatePinnedToCore(lidarTask, "lidar", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(hoverboardTask, "hover", 4096, NULL, 3, NULL, 0);
  webDriveBegin();   // core 1, below loop() in importance

  wifiCfgBegin();    // NVS credentials before the first WiFi.begin()
  wifiBegin();
}

// ----------------------------------------------------------------------- loop
void loop() {
  wifiLoop();
  otaLoop();          // blocks here for the whole upload, then reboots
  sendPendingScan();  // LDS2 to the owner (core 1: a WiFi send can stall)
  delay(2);
}
