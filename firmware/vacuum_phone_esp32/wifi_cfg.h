// wifi_cfg.h - runtime Wi-Fi credentials (NVS), chosen from the phone app.
//
// secrets.h stays the compile-time DEFAULT (bench bring-up with no app);
// WFC1 (docs/PROTOCOL.md §3.9) writes SSID+password into NVS and the board
// joins the network from the next wifiLoop() pass. NVS survives OTA reboots
// and power cycles. WFC1 with an empty SSID clears NVS -> back to secrets.h.
//
// Threading: phHandleDatagram runs on core 0 (hoverboardTask) or the BLE
// task; wifiLoop runs on core 1. The request crosses with a portMUX-guarded
// buffer + flag (same pattern as ph_owner in phone_link.h): fill, set the
// flag under the lock, consume under the lock.

#pragma once

#include <Arduino.h>
#include <Preferences.h>

#include "secrets.h"   // compile-time defaults (git-ignored)

static const uint8_t WIFI_CFG_SSID_MAX = 32;   // 802.11 SSID limit
static const uint8_t WIFI_CFG_PASS_MAX = 64;   // WPA2 passphrase limit

static portMUX_TYPE wifi_cfg_mux = portMUX_INITIALIZER_UNLOCKED;
static char wifi_cfg_ssid[WIFI_CFG_SSID_MAX + 1] = WIFI_SSID;
static char wifi_cfg_pass[WIFI_CFG_PASS_MAX + 1] = WIFI_PASS;
static bool wifi_cfg_pending = false;          // a WFC1 waits for wifiLoop()
static bool wifi_cfg_clear = false;            // empty SSID = revert to default
static char wifi_cfg_new_ssid[WIFI_CFG_SSID_MAX + 1];
static char wifi_cfg_new_pass[WIFI_CFG_PASS_MAX + 1];
static uint32_t wifi_cfg_request_ms = 0;       // delay apply so the WFCA flushes

// Boot: load NVS credentials, else keep the compile-time secrets.h pair.
static void wifiCfgBegin() {
  Preferences p;
  if (p.begin("vacuum", true /* read-only */)) {
    const String s = p.getString("ssid", "");
    const String pw = p.getString("pass", "");
    p.end();
    if (s.length() > 0 && s.length() <= WIFI_CFG_SSID_MAX) {
      strlcpy(wifi_cfg_ssid, s.c_str(), sizeof(wifi_cfg_ssid));
      strlcpy(wifi_cfg_pass, pw.c_str(), sizeof(wifi_cfg_pass));
      Serial.printf("wifi_cfg: using NVS network \"%s\"\n", wifi_cfg_ssid);
      return;
    }
  }
  Serial.printf("wifi_cfg: default network \"%s\" (secrets.h)\n", wifi_cfg_ssid);
}

// phHandleDatagram (core 0 / BLE task): stage the new credentials. The
// reply (WFCA) goes out first; wifiLoop applies >=100 ms later so the ack
// is already on the wire before WiFi.disconnect() can drop it.
static void wifiCfgRequest(const uint8_t * ssid, uint8_t sl,
                           const uint8_t * pass, uint8_t pl, bool clear) {
  portENTER_CRITICAL(&wifi_cfg_mux);
  if (clear) {
    wifi_cfg_new_ssid[0] = '\0';
    wifi_cfg_new_pass[0] = '\0';
  } else {
    memcpy(wifi_cfg_new_ssid, ssid, sl);
    wifi_cfg_new_ssid[sl] = '\0';
    memcpy(wifi_cfg_new_pass, pass, pl);
    wifi_cfg_new_pass[pl] = '\0';
  }
  wifi_cfg_clear = clear;
  wifi_cfg_request_ms = millis();
  wifi_cfg_pending = true;
  portEXIT_CRITICAL(&wifi_cfg_mux);
}

// wifiLoop (core 1): consume a pending request. Saves to NVS and updates
// the live buffers; returns true when the caller should WiFi.disconnect()
// + WiFi.begin() with wifiCfgSsid()/wifiCfgPass(). False while the100 ms
// ack-grace window (or when nothing is pending).
static bool wifiCfgApplyPending() {
  bool run = false;
  portENTER_CRITICAL(&wifi_cfg_mux);
  if (wifi_cfg_pending &&
      (uint32_t)(millis() - wifi_cfg_request_ms) >= 100) {
    wifi_cfg_pending = false;
    run = true;
    if (wifi_cfg_clear) {
      wifi_cfg_ssid[0] = '\0';
      wifi_cfg_pass[0] = '\0';
    } else {
      strlcpy(wifi_cfg_ssid, wifi_cfg_new_ssid, sizeof(wifi_cfg_ssid));
      strlcpy(wifi_cfg_pass, wifi_cfg_new_pass, sizeof(wifi_cfg_pass));
    }
  }
  portEXIT_CRITICAL(&wifi_cfg_mux);
  if (!run) return false;

  Preferences p;
  if (p.begin("vacuum", false /* read-write */)) {
    if (wifi_cfg_clear) {
      p.remove("ssid");
      p.remove("pass");
      Serial.println("wifi_cfg: cleared, back to secrets.h defaults");
    } else {
      p.putString("ssid", wifi_cfg_ssid);
      p.putString("pass", wifi_cfg_pass);
      Serial.printf("wifi_cfg: saved \"%s\" to NVS\n", wifi_cfg_ssid);
    }
    p.end();
  } else {
    Serial.println("wifi_cfg: NVS open failed (kept in RAM only)");
  }
  return true;
}

// What wifiBegin()/retries should join: NVS (or secrets.h when unset).
static const char *wifiCfgSsid() { return wifi_cfg_ssid; }
static const char *wifiCfgPass() { return wifi_cfg_pass; }
