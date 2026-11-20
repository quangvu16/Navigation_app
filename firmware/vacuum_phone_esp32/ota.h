// ota.h - firmware updates over Wi-Fi (ArduinoOTA, part of the ESP32 core).
//
//   tools/ota_upload.sh <sketch-dir> hoverboard.local     (or the board's IP)
//
// Three rules, because this board drives a hoverboard:
//   - A password is required (OTA_PASSWORD in secrets.h). Without one, OTA
//     stays off: anyone on the Wi-Fi could otherwise flash the robot.
//   - Requests are only answered while the robot stands still (nothing
//     commanded, wheels not turning). While it moves, espota times out with
//     "No response from the ESP"; stop the robot and try again.
//   - From the first byte to the reboot, the wheels are held at zero and
//     every command (ROS, web stick, sonar reflex) is ignored.
//
// The upload runs inside loop() on core 1 and blocks it for 10-30 s: no LiDAR
// scans go out meanwhile. hoverboardTask on core 0 keeps feeding the
// mainboard zero commands. On success the board reboots into the new image;
// the bootloader falls back to the old one if the new one does not boot.

#pragma once

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <HTTPUpdate.h>
#include <NetworkClient.h>
#include <NetworkClientSecure.h>

#include "config.h"

// Written by hoverboardTask (core 0), read here (core 1). Single bools, so no
// lock: a stale read costs one 2 ms loop() pass.
static volatile bool ota_robot_still = true;
// Set when an upload starts; hoverboardTask then holds the wheels at zero.
static volatile bool ota_active = false;

static bool ota_ready = false;      // begin() ran with a password

static bool otaEnabled() {
#if OTA_ENABLED
  return strlen(OTA_PASSWORD) > 0;
#else
  return false;
#endif
}

// ------------------------------------------------------- URL OTA (WFO1)
// The app sends a firmware URL (docs/PROTOCOL.md §3.10); the download runs
// HERE in loop() on core 1 - never in hoverboardTask, which must keep
// feeding the mainboard. Same safety contract as the ArduinoOTA path:
// password checked on arrival, robot must stand still, wheels held at zero
// (ota_active) from accept to reboot. A failed download leaves the old
// firmware running and reports back through otaNotifyOwner().
static char ota_url[208];
static volatile bool ota_url_pending = false;
static volatile bool ota_url_running = false;

// Declared here (ota.h is included before phSendTo exists), defined by
// phone_link.h: sends WFOA/WFOB to the owner.
static void otaNotifyOwner(uint8_t status, uint8_t detail);

// phHandleDatagram (core 0 / BLE task): stage the download.
// Returns the WFOA status the caller should reply with.
static uint8_t otaUrlRequest(const char * url, uint8_t url_len,
                             const char * pass, uint8_t pass_len) {
  if (!otaEnabled()) return 5;                    // OTA disabled / no password
  if (strlen(OTA_PASSWORD) == 0 || pass_len == 0 ||
      memcmp(pass, OTA_PASSWORD, pass_len) != 0 ||
      strlen(OTA_PASSWORD) != pass_len) return 2; // wrong OTA password
  if (url_len < 8 || url_len >= (uint8_t)sizeof(ota_url)) return 4;
  if (strncmp(url, "http://", 7) != 0 &&
      strncmp(url, "https://", 8) != 0) return 4; // only http(s) URLs
  if (!ota_robot_still) return 3;                 // stop the robot first
  if (ota_url_pending || ota_url_running || ota_active) return 7; // busy
  memcpy(ota_url, url, url_len);
  ota_url[url_len] = '\0';
  ota_url_pending = true;
  return 0;                                       // accepted
}

// loop() (core 1): run a staged download when the robot stands still.
static void otaUrlRun() {
  if (!ota_url_pending || ota_url_running || ota_active) return;
  if (!ota_robot_still) return;                   // wait for the wheels
  ota_url_pending = false;
  ota_url_running = true;
  ota_active = true;                              // hoverboardTask: wheels to 0
  Serial.printf("ota: fetching %s\n", ota_url);

  httpUpdate.rebootOnUpdate(false);               // report WFOB before reboot
  HTTPUpdateResult ret;
  if (strncmp(ota_url, "https://", 8) == 0) {
    NetworkClientSecure client;
    // No CA bundle on the board: transport is encrypted but not pinned.
    // Trust comes from the owner lease + OTA password on WFO1 itself.
    client.setInsecure();
    ret = httpUpdate.update(client, ota_url);
  } else {
    NetworkClient client;
    ret = httpUpdate.update(client, ota_url);
  }

  switch (ret) {
    case HTTP_UPDATE_OK:
      Serial.println("ota: image written, rebooting");
      otaNotifyOwner(0, 0);                       // WFOB success
      ota_url_running = false;
      delay(300);                                 // let the datagram flush
      ESP.restart();
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("ota: no update available at that URL");
      otaNotifyOwner(2, 0);
      break;
    default:                                      // HTTP_UPDATE_FAILED
      Serial.printf("ota: failed (%s) - old firmware keeps running\n",
                    httpUpdate.getLastErrorString().c_str());
      otaNotifyOwner(1, (uint8_t)constrain(httpUpdate.getLastError(), 0, 255));
      break;
  }
  ota_active = false;
  ota_url_running = false;
}

// Once, from loop(), the first time WiFi is up. mDNS is handled by the caller
// (one hostname for the drive page and OTA).
static void otaBegin() {
#if OTA_ENABLED
  if (ota_ready) return;
  if (!otaEnabled()) {
    Serial.println("ota: OFF - add #define OTA_PASSWORD \"...\" to secrets.h to enable it");
    return;
  }
  ArduinoOTA.setHostname(ROBOT_HOSTNAME);
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.setMdnsEnabled(false);

  ArduinoOTA.onStart([]() {
    ota_active = true;            // hoverboardTask zeroes the wheels from now on
    Serial.printf("ota: receiving %s - wheels held at zero\n",
                  ArduinoOTA.getCommand() == U_FLASH ? "firmware" : "filesystem");
  });
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    static int last = -1;
    const int pct = total ? (int)(100ULL * done / total) : 0;
    if (pct / 10 != last) { last = pct / 10; Serial.printf("ota: %d%%\n", pct); }
  });
  ArduinoOTA.onEnd([]() { Serial.println("ota: done, rebooting"); });
  ArduinoOTA.onError([](ota_error_t e) {
    static const char * const NAME[] = {"auth failed (wrong password)", "begin failed (image too big?)",
                                        "connect failed", "receive failed", "end failed (bad image?)"};
    Serial.printf("ota: error: %s - old firmware keeps running\n", (unsigned)e < 5 ? NAME[e] : "unknown");
    ota_active = false;
  });
  ArduinoOTA.begin();
  ota_ready = true;
  Serial.printf("ota: ready on %s.local:%u (password set; answers only while the robot stands still)\n",
                ROBOT_HOSTNAME, (unsigned)OTA_PORT);
#endif
}

// Every loop() pass.
static void otaLoop() {
  otaUrlRun();   // staged WFO1 download (no-op unless one is pending)
#if OTA_ENABLED
  if (ota_ready && ota_robot_still) ArduinoOTA.handle();
#endif
}
