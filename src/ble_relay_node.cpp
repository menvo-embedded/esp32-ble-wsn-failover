// ble_relay_node.cpp — NODE CHUYEN TIEP (Relay Node / Node 2)
// Vai tro BLE: DUAL-ROLE, chay 2 vai cung luc:
//   (1) GATT Client -> Gateway: quet tim GATEWAY_SERVICE_UUID, ket noi, ghi goi vao RxData
//   (2) GATT Server <- Sensor : mo RelayRxData (WRITE) de Sensor ghi goi vao khi dang o che do Relay
// Nguyen tac: Relay CHI quang cao (cho Sensor ket noi) khi dang co duong len Gateway ("uplink").
//   Mat uplink -> ngung quang cao + ngat Sensor, de Sensor khong nhan ACK "gia" tu 1 Relay
//   khong chuyen tiep duoc (ACK theo tung chang: ACK Sensor nhan duoc la ACK cua Relay).
// Build + nap: pio run -e relay -t upload --upload-port COMx
// LUU Y: node nay KHONG duoc Deep Sleep (tat radio -> khong nhan duoc goi relay).
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "wsn_protocol.h"

#define DEVICE_NAME        "WSN-Relay"
#define SCAN_TIME_MS       5000
#define SUMMARY_PERIOD_MS  10000

// ================= Hang doi goi nhan tu Sensor =================
// onWrite() (task BLE) chi bo goi vao hang doi; loop() lay ra va Write sang Gateway.
// Write sang Gateway ngay trong onWrite() = goi ham BLE blocking trong callback -> DEADLOCK.
struct RxItem {
  Packet   pkt;
  uint32_t rxMillis;
};
QueueHandle_t gRxQueue = nullptr;

// ================= Phan Client (uplink -> Gateway) =================
volatile bool gDoConnect = false;
volatile bool gNeedScan  = true;
volatile bool gUplinkLost = false;   // callback bao cho loop() xu ly viec mat uplink
NimBLEAddress gGatewayAddr;
NimBLEClient *pUplink = nullptr;
NimBLERemoteCharacteristic *pGatewayRxData = nullptr;

// ================= Phan Server (downlink <- Sensor) =================
NimBLEServer *pServer = nullptr;

// ================= Thong ke =================
uint32_t gRxFromSensor = 0;   // so goi nhan tu Sensor
uint32_t gFwdOk = 0;          // chuyen tiep thanh cong (Gateway ACK)
uint32_t gFwdFail = 0;        // chuyen tiep that bai
uint32_t gDropTtl = 0;        // bo vi TTL = 0
uint32_t gDropNoUplink = 0;   // bo vi chua co uplink
uint32_t gBadSize = 0;

bool uplinkReady() {
  return pUplink != nullptr && pUplink->isConnected() && pGatewayRxData != nullptr;
}

// ================= Callback phia Server (Sensor ket noi toi Relay) =================
class RelayServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, NimBLEConnInfo &connInfo) override {
    Serial.printf("[DOWN] + Sensor %s ket noi (interval=%.2f ms) | so ket noi server: %u\n",
                  connInfo.getAddress().toString().c_str(),
                  connInfo.getConnInterval() * 1.25f, s->getConnectedCount());
  }

  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &connInfo, int reason) override {
    Serial.printf("[DOWN] - Sensor %s ngat, reason=0x%02X\n",
                  connInfo.getAddress().toString().c_str(), reason);
    // advertiseOnDisconnect(false) -> loop() tu quyet dinh co quang cao lai hay khong
  }
};

class RelayRxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override {
    NimBLEAttValue v = pChar->getValue();
    if (v.size() != sizeof(Packet)) {
      gBadSize++;
      Serial.printf("[DOWN] !! sai kich thuoc: %u byte\n", v.size());
      return;
    }
    RxItem item;
    memcpy(&item.pkt, v.data(), sizeof(Packet));
    item.rxMillis = millis();
    if (xQueueSend(gRxQueue, &item, 0) != pdTRUE) {
      Serial.println("[DOWN] !! hang doi day, bo goi");
    }
  }
};

// ================= Callback quet (tim Gateway) =================
class RelayScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *dev) override {
    if (!dev->isAdvertisingService(NimBLEUUID(GATEWAY_SERVICE_UUID))) return;
    Serial.printf("[SCAN] Tim thay Gateway %s rssi=%d\n",
                  dev->getAddress().toString().c_str(), dev->getRSSI());
    gGatewayAddr = dev->getAddress();
    gDoConnect = true;
    NimBLEDevice::getScan()->stop();
  }

  void onScanEnd(const NimBLEScanResults &results, int reason) override {
    if (!gDoConnect) {
      Serial.println("[SCAN] Khong thay Gateway -> quet lai");
      gNeedScan = true;
    }
  }
};

// ================= Callback phia Client (uplink) =================
class UplinkCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *c) override {
    Serial.printf("[UP] Da ket noi Gateway %s\n", c->getPeerAddress().toString().c_str());
  }
  void onConnectFail(NimBLEClient *c, int reason) override {
    Serial.printf("[UP] !! Ket noi Gateway that bai, reason=0x%02X\n", reason);
    gNeedScan = true;
  }
  void onDisconnect(NimBLEClient *c, int reason) override {
    Serial.printf("[UP] - Mat ket noi Gateway, reason=0x%02X\n", reason);
    pGatewayRxData = nullptr;
    gUplinkLost = true;
    gNeedScan = true;
  }
};

// -----------------------------------------------------------------
void startScan() {
  gNeedScan = false;
  gDoConnect = false;
  Serial.println("[SCAN] Dang quet tim Gateway...");
  NimBLEDevice::getScan()->start(SCAN_TIME_MS, false, true);
}

// Mo / dong "cua" cho Sensor: chi quang cao khi co uplink
void openDownlink() {
  if (NimBLEDevice::startAdvertising()) {
    Serial.println("[DOWN] Bat quang cao -> Sensor co the ket noi qua Relay");
  }
}

void closeDownlink() {
  NimBLEDevice::stopAdvertising();
  for (uint16_t h : pServer->getPeerDevices()) {
    pServer->disconnect(h);
  }
  Serial.println("[DOWN] Tat quang cao + ngat Sensor (khong co uplink)");
}

bool connectToGateway() {
  if (pUplink == nullptr) {
    pUplink = NimBLEDevice::createClient();
    pUplink->setClientCallbacks(new UplinkCallbacks(), false);
    // Interval 50-100 ms giua cac chang de tranh dung do connection event (bao cao muc 3.2)
    pUplink->setConnectionParams(40, 80, 0, 200);
    pUplink->setConnectTimeout(5000);
  }

  Serial.printf("[UP] Dang ket noi %s ...\n", gGatewayAddr.toString().c_str());
  if (!pUplink->connect(gGatewayAddr)) return false;

  NimBLERemoteService *svc = pUplink->getService(GATEWAY_SERVICE_UUID);
  pGatewayRxData = svc ? svc->getCharacteristic(GATEWAY_RXDATA_UUID) : nullptr;
  if (pGatewayRxData == nullptr || !pGatewayRxData->canWrite()) {
    Serial.println("[UP] !! Khong tim thay RxData cua Gateway -> ngat");
    pGatewayRxData = nullptr;
    pUplink->disconnect();
    return false;
  }
  Serial.printf("[UP] Uplink san sang | interval=%.2f ms\n",
                pUplink->getConnInfo().getConnInterval() * 1.25f);
  return true;
}

void forwardPacket(const RxItem &item) {
  Packet pkt = item.pkt;
  gRxFromSensor++;

  if (pkt.ttl == 0) {
    gDropTtl++;
    Serial.printf("[FWD] node=0x%04X seq=%u TTL=0 -> BO (chong vong lap)\n", pkt.nodeId, pkt.seq);
    return;
  }
  if (!uplinkReady()) {
    gDropNoUplink++;
    Serial.printf("[FWD] node=0x%04X seq=%u -> BO (chua co uplink)\n", pkt.nodeId, pkt.seq);
    return;
  }

  pkt.ttl--;
  pkt.relayFlag = RELAY_FLAG_RELAYED;

  uint32_t t0 = millis();
  bool ok = pGatewayRxData->writeValue((uint8_t *)&pkt, sizeof(pkt), true);  // With Response
  uint32_t rtt = millis() - t0;

  if (ok) gFwdOk++; else gFwdFail++;
  Serial.printf("[FWD] node=0x%04X seq=%u value=%.2f ttl=%u -> Gateway %s (%lu ms, cho %lu ms trong hang doi)\n",
                pkt.nodeId, pkt.seq, pkt.value, pkt.ttl, ok ? "ACK OK" : "ACK FAIL",
                rtt, t0 - item.rxMillis);
}

void printSummary() {
  Serial.printf("[SUM] t=%lus uplink=%s sensor_ket_noi=%u | nhan=%lu fwd_ok=%lu fwd_fail=%lu "
                "bo_ttl=%lu bo_khong_uplink=%lu sai_kich_thuoc=%lu\n",
                millis() / 1000, uplinkReady() ? "CO" : "KHONG", pServer->getConnectedCount(),
                gRxFromSensor, gFwdOk, gFwdFail, gDropTtl, gDropNoUplink, gBadSize);
}

// =================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n===== WSN RELAY NODE (Dual-Role) =====");

  gRxQueue = xQueueCreate(16, sizeof(RxItem));

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(9);
  Serial.printf("[INIT] Dia chi BLE: %s\n", NimBLEDevice::getAddress().toString().c_str());

  // ---- Vai Server: RelayRxData ----
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new RelayServerCallbacks());
  pServer->advertiseOnDisconnect(false);   // tu quan ly quang cao theo trang thai uplink
  NimBLEService *svc = pServer->createService(RELAY_SERVICE_UUID);
  NimBLECharacteristic *pRx = svc->createCharacteristic(RELAY_RXDATA_UUID, NIMBLE_PROPERTY::WRITE);
  pRx->setCallbacks(new RelayRxCallbacks());
  // BAT BUOC khoi dong GATT server NGAY BAY GIO (chua co ket noi nao).
  // Neu de NimBLE tu khoi dong luc bat quang cao (sau khi da ket noi uplink toi Gateway)
  // -> ble_gatts_reset() that bai vi dang co ket noi -> "assert failed: ble_svc_gap_init" -> reboot.
  if (!pServer->start()) {
    Serial.println("[INIT] !! Khong khoi dong duoc GATT server");
  }

  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  pAdv->enableScanResponse(true);
  pAdv->setName(DEVICE_NAME);
  pAdv->addServiceUUID(RELAY_SERVICE_UUID);
  // CHUA quang cao: doi co uplink roi moi mo

  // ---- Vai Client: quet tim Gateway ----
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new RelayScanCallbacks(), false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(100);

  Serial.printf("[INIT] Relay service %s\n", RELAY_SERVICE_UUID);
  Serial.println("[INIT] Cho ket noi Gateway truoc khi mo cho Sensor...");
}

void loop() {
  // 1) Tim thay Gateway -> ket noi uplink -> mo cua cho Sensor
  if (gDoConnect) {
    gDoConnect = false;
    if (connectToGateway()) {
      gUplinkLost = false;
      openDownlink();
    } else {
      gNeedScan = true;
    }
  }

  // 2) Mat uplink -> dong cua cho Sensor
  if (gUplinkLost) {
    gUplinkLost = false;
    closeDownlink();
  }

  // 2b) Co uplink nhung khong co Sensor nao ket noi va dang khong quang cao
  //     (vd Sensor vua ngat de thu quay ve Gateway) -> mo cua lai de Sensor quay lai duoc.
  //     Can lam thu cong vi da tat advertiseOnDisconnect trong setup().
  if (uplinkReady() && pServer->getConnectedCount() == 0 &&
      !NimBLEDevice::getAdvertising()->isAdvertising()) {
    openDownlink();
  }

  // 3) Quet lai Gateway khi can
  if (gNeedScan && !uplinkReady() && !NimBLEDevice::getScan()->isScanning()) {
    startScan();
  }

  // 4) Chuyen tiep cac goi dang cho
  RxItem item;
  while (xQueueReceive(gRxQueue, &item, 0) == pdTRUE) {
    forwardPacket(item);
  }

  // 5) Lenh qua Serial: 's' = in tong ket ngay
  while (Serial.available()) {
    if (Serial.read() == 's') printSummary();
  }

  // 6) Tong ket dinh ky, nhung CHI in khi so lieu thay doi (khong co gi xay ra -> log dung yen)
  static uint32_t lastSum = 0, lastSig = 0;
  if (millis() - lastSum >= SUMMARY_PERIOD_MS) {
    lastSum = millis();
    uint32_t sig = gRxFromSensor + gFwdOk + gFwdFail + gDropTtl + gDropNoUplink + gBadSize +
                   pServer->getConnectedCount() + (uplinkReady() ? 1000 : 0);
    if (sig != lastSig) {
      lastSig = sig;
      printSummary();
    }
  }

  delay(5);
}
