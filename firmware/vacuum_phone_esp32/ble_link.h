// ble_link.h - Bluetooth Low Energy transport for the phone link.
//
// Ported from the FriendBot ESP32-S3 project
// ("/Volumes/0775999222/App/FriendBot ESP32S3 Xiaozhi 2W/firmware/
// openbot-esp32s3/openbot-esp32s3.ino"), which proves this exact GATT
// layout with its phone apps: a UART-style service with a write-only RX
// characteristic (phone -> board) and a notify TX characteristic
// (board -> phone). The UUIDs are FriendBot's verbatim so tooling that
// already knows them keeps working.
//
// Framing differs from FriendBot's newline text protocol on purpose: our
// v2 datagrams are binary and GATT writes are message boundaries already,
// so ONE write = ONE datagram (HELO/HBC2/PNG1) fed into the same
// phHandleDatagram() the UDP path uses - claim, lease, E-stop and the
// 300 ms watchdog behave identically on both transports.
//
// TX notify framing (parsed by the app's BleRobotLink):
//   [0x00][datagram...]        - one whole packet (HBT2/HACK/PNG2/BYE1)
//   [0x01][idx][count][slice]  - LDS2 split for the BLE MTU
//
// Why BLE at all (FriendBot's lesson): no router, no AP mode, instant
// connection within ~10 m, and it keeps working when Wi-Fi does not
// (riding on the robot, outdoors, crowded 2.4 GHz). Wi-Fi stays the
// high-bandwidth path; BLE is the reliable control path.

#pragma once

#include <Arduino.h>
#if __has_include(<BLEDevice.h>)
#define PH_HAS_BLE 1
#else
#define PH_HAS_BLE 0
#endif

#if PH_HAS_BLE

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "phone_link.h"

// FriendBot's UUIDs verbatim (UART-style service).
static const char *PH_BLE_SERVICE_UUID =
    "61653dc3-4021-4d1e-ba83-8b4eec61d613";
static const char *PH_BLE_RX_UUID =
    "06386c14-86ea-4d71-811c-48f97c58f8c9";   // phone writes commands
static const char *PH_BLE_TX_UUID =
    "9bf1103b-834c-47cf-b149-c9e4bcf778a7";   // board notifies telemetry

static BLEServer *ph_ble_server = nullptr;
static BLECharacteristic *ph_ble_tx = nullptr;
static volatile bool ph_ble_client = false;

static bool phBleNotifyImpl(const uint8_t *buf, uint16_t len) {
  if (!ph_ble_tx || !ph_ble_client) return false;
  ph_ble_tx->setValue(const_cast<uint8_t *>(buf), len);
  ph_ble_tx->notify();
  return true;
}
static bool phBleConnectedImpl() { return ph_ble_client; }

class PhBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    ph_ble_client = true;
    Serial.println("phone_link: BLE client connected");
  }
  void onDisconnect(BLEServer *) override {
    ph_ble_client = false;
    Serial.println("phone_link: BLE client disconnected");
    // FriendBot restarts advertising from the loop; we do it here so a
    // dropped connection never needs the main loop to notice.
    ph_ble_server->startAdvertising();
    Serial.println("phone_link: BLE advertising again");
  }
};

class PhBleRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    const std::string v = c->getValue();
    if (v.empty()) return;
    // One GATT write = one datagram (HELO / HBC2 / PNG1 ...).
    phHandleBle(reinterpret_cast<const uint8_t *>(v.data()),
                static_cast<int>(v.length()));
  }
};

static void phBleBegin() {
  BLEDevice::init(ROBOT_HOSTNAME);
  BLEDevice::setMTU(517);   // LDS2 chunks fit comfortably below this
  ph_ble_server = BLEDevice::createServer();
  ph_ble_server->setCallbacks(new PhBleServerCallbacks());

  BLEService *service =
      ph_ble_server->createService(BLEUUID(PH_BLE_SERVICE_UUID));

  ph_ble_tx = service->createCharacteristic(
      BLEUUID(PH_BLE_TX_UUID),
      BLECharacteristic::PROPERTY_NOTIFY);
  ph_ble_tx->addDescriptor(new BLE2902());   // CCCD: enable notifications

  BLECharacteristic *rx = service->createCharacteristic(
      BLEUUID(PH_BLE_RX_UUID),
      BLECharacteristic::PROPERTY_WRITE_NR);  // write-without-response, fast
  rx->setCallbacks(new PhBleRxCallbacks());

  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLEUUID(PH_BLE_SERVICE_UUID));
  adv->setScanResponse(true);
  // FriendBot uses the defaults; low latency matters more than power here
  // (the board is mains/24 V powered).
  adv->setMinInterval(6);    // 7.5 ms units -> ~45 ms connection interval
  adv->setMaxInterval(48);
  BLEDevice::startAdvertising();

  // Install the transport hooks phone_link.h / lidar_ld14.h call.
  ph_ble_notify = phBleNotifyImpl;
  ph_ble_connected_fn = phBleConnectedImpl;

  Serial.printf("phone_link: BLE up, service %s (RX %s / TX %s)\n",
                PH_BLE_SERVICE_UUID, PH_BLE_RX_UUID, PH_BLE_TX_UUID);
}

#else  // no BLE headers in this core build

static void phBleBegin() {
  Serial.println("phone_link: BLE headers not available in this core");
}

#endif  // PH_HAS_BLE
