// ble_gateway_node.cpp — NODE TRUNG TAM (Gateway)
// Vai tro BLE: Peripheral / GATT Server.
// Mo characteristic RxData (WRITE) de Sensor Node va Relay Node ghi goi tin Packet (10 byte) vao.
// ACK: client dung Write With Response -> NimBLE tu tra Write Response, Gateway khong can code gi them.
// Build + nap: pio run -e gateway -t upload --upload-port COMx
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "wsn_protocol.h"

#define DEVICE_NAME        "WSN-Gateway"
#define MAX_NODES          8        // so Node_ID toi da Gateway theo doi thong ke
#define RSSI_PERIOD_MS     1000     // chu ky doc RSSI cac ket noi
#define SUMMARY_PERIOD_MS  10000    // chu ky in bang tong ket

// Dia chi "random static" tu dat thay cho dia chi phan cung (b0:cb:d8:ce:8b:4e).
// Ly do: dien thoai da tung ghep doi (bond) voi dia chi phan cung khi board chay chuong trinh cu,
// giu khoa ma hoa cu -> ket noi nao cung bi dien thoai ngat ("Encryption keys are missing").
// Doi dia chi -> dien thoai coi la thiet bi moi. 2 bit cao cua byte dau phai la 11 (0xC0..0xFF).
// Neu sau nay lai bi loi bond tuong tu, chi can doi byte cuoi (vd :01 -> :02).
#define GATEWAY_STATIC_ADDR "C0:DE:57:A7:10:01"

// ================= Hang doi goi nhan =================
// onWrite() chay tren task cua BLE stack. Chi copy goi vao hang doi FreeRTOS roi thoat ngay;
// loop() (task Arduino) lay ra xu ly + in log. Dung hang doi (khong dung 1 bien co) vi Sensor
// va Relay co the ghi gan nhu cung luc -> 1 bien co se bi ghi de, mat goi.
struct RxItem {
  Packet   pkt;
  uint16_t connHandle;   // ket noi nao da ghi (Sensor truc tiep hay Relay)
  uint32_t rxMillis;
};
QueueHandle_t gRxQueue = nullptr;

// ================= Thong ke theo tung Node_ID =================
struct NodeStats {
  bool     used;
  uint16_t nodeId;
  uint16_t lastSeq;
  uint32_t recv;       // so goi hop le da nhan
  uint32_t lost;       // so goi mat (suy ra tu khoang trong seq)
  uint32_t dup;        // goi trung / goi cu den tre
  uint32_t relayed;    // so goi di qua Relay (relayFlag = 1)
};
NodeStats gStats[MAX_NODES];
uint32_t  gBadSize = 0;   // so lan nhan goi sai kich thuoc

// ================= RSSI tung ket noi =================
// Doc trong loop() (an toan), KHONG doc trong callback (xem canh bao deadlock trong CLAUDE.md).
struct ConnRssi {
  uint16_t connHandle;
  int8_t   rssi;
};
ConnRssi gRssi[CONFIG_BT_NIMBLE_MAX_CONNECTIONS];

NimBLEServer *pServer = nullptr;

// -----------------------------------------------------------------
NodeStats *getStats(uint16_t nodeId) {
  for (auto &s : gStats) {
    if (s.used && s.nodeId == nodeId) return &s;
  }
  for (auto &s : gStats) {
    if (!s.used) {
      s = {};
      s.used = true;
      s.nodeId = nodeId;
      return &s;
    }
  }
  return nullptr;  // het cho
}

int8_t getCachedRssi(uint16_t connHandle) {
  for (auto &r : gRssi) {
    if (r.connHandle == connHandle) return r.rssi;
  }
  return 0;  // chua co so lieu
}

// ================= Callback ket noi =================
class GatewayServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, NimBLEConnInfo &connInfo) override {
    Serial.printf("[CONN] + %s (handle=%u, interval=%.2f ms) | dang ket noi: %u\n",
                  connInfo.getAddress().toString().c_str(),
                  connInfo.getConnHandle(),
                  connInfo.getConnInterval() * 1.25f,
                  s->getConnectedCount());
    // Quang cao tu dung khi co ket noi -> bat lai de node thu 2 (Relay) van ket noi duoc
    if (s->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
      NimBLEDevice::startAdvertising();
    }
  }

  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &connInfo, int reason) override {
    Serial.printf("[CONN] - %s (handle=%u) reason=0x%02X | con lai: %u\n",
                  connInfo.getAddress().toString().c_str(),
                  connInfo.getConnHandle(),
                  reason,
                  s->getConnectedCount());
    // NimBLE 2.x mac dinh advertiseOnDisconnect = true -> tu quang cao lai
  }

  void onMTUChange(uint16_t mtu, NimBLEConnInfo &connInfo) override {
    Serial.printf("[CONN] MTU = %u (handle=%u)\n", mtu, connInfo.getConnHandle());
  }
};

// ================= Callback khi co node Write vao RxData =================
class RxDataCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override {
    NimBLEAttValue v = pChar->getValue();
    if (v.size() != sizeof(Packet)) {
      gBadSize++;
      Serial.printf("[RX] !! sai kich thuoc: %u byte (can %u) tu handle=%u\n",
                    v.size(), sizeof(Packet), connInfo.getConnHandle());
      return;
    }
    RxItem item;
    memcpy(&item.pkt, v.data(), sizeof(Packet));
    item.connHandle = connInfo.getConnHandle();
    item.rxMillis = millis();
    if (xQueueSend(gRxQueue, &item, 0) != pdTRUE) {
      Serial.println("[RX] !! hang doi day, bo goi");
    }
  }
};

// ================= Xu ly 1 goi (chay trong loop) =================
void handlePacket(const RxItem &item) {
  const Packet &p = item.pkt;
  NodeStats *st = getStats(p.nodeId);
  if (st == nullptr) {
    Serial.printf("[RX] !! qua %d node, bo qua node 0x%04X\n", MAX_NODES, p.nodeId);
    return;
  }

  // Tinh mat goi tu khoang trong seq (uint16 tu quay vong 65535 -> 0)
  const char *note = "";
  if (st->recv == 0 && st->dup == 0) {
    note = "first";
  } else {
    uint16_t diff = (uint16_t)(p.seq - st->lastSeq);
    if (diff == 0) {
      st->dup++;
      note = "DUP";
    } else if (diff < 0x8000) {
      if (diff > 1) {
        st->lost += diff - 1;
        note = "GAP";
      }
    } else {
      // seq lui lai nhieu -> node vua reset (seq bat dau lai tu 0) hoac goi cu den tre
      note = "RESET";
    }
  }

  if (strcmp(note, "DUP") != 0) {
    st->lastSeq = p.seq;
    st->recv++;
    if (p.relayFlag == RELAY_FLAG_RELAYED) st->relayed++;
  }

  int8_t rssi = getCachedRssi(item.connHandle);

  // Dong log de doc
  Serial.printf("[RX] node=0x%04X seq=%u value=%.2f %s ttl=%u via=handle%u rssi=%d %s\n",
                p.nodeId, p.seq, p.value,
                p.relayFlag == RELAY_FLAG_RELAYED ? "RELAY " : "DIRECT",
                p.ttl, item.connHandle, rssi, note);

  // Dong CSV de copy ra file phan tich
  // CSV,millis,nodeId,seq,value,relayFlag,ttl,rssi_dBm(chang cuoi),recv,lost,dup
  Serial.printf("CSV,%lu,%u,%u,%.2f,%u,%u,%d,%lu,%lu,%lu\n",
                item.rxMillis, p.nodeId, p.seq, p.value, p.relayFlag, p.ttl, rssi,
                st->recv, st->lost, st->dup);
}

// ================= Doc RSSI cac ket noi (chay trong loop) =================
void updateRssi() {
  for (auto &r : gRssi) r = {BLE_HS_CONN_HANDLE_NONE, 0};
  // Lay danh sach handle cac ket noi dang co. KHONG dung getPeerInfo(index): khi 1 node ngat,
  // o thu tu cua no bi trong -> getPeerInfo(0) bao "Peer info not found" du van con node khac.
  uint8_t i = 0;
  for (uint16_t h : pServer->getPeerDevices()) {
    if (i >= CONFIG_BT_NIMBLE_MAX_CONNECTIONS) break;
    int8_t rssi = 0;
    // Bo gia tri vo ly (vd -122, 127 = "khong co so lieu"): ESP32 chi thu duoc toi ~-100 dBm
    if (ble_gap_conn_rssi(h, &rssi) == 0 && rssi >= -110 && rssi <= 20) {
      gRssi[i++] = {h, rssi};
    }
  }
}

void printSummary() {
  Serial.printf("[SUM] t=%lus ket_noi=%u sai_kich_thuoc=%lu\n",
                millis() / 1000, pServer->getConnectedCount(), gBadSize);
  for (auto &s : gStats) {
    if (!s.used) continue;
    uint32_t total = s.recv + s.lost;
    float pdr = total ? 100.0f * s.recv / total : 0;
    Serial.printf("[SUM]   node=0x%04X recv=%lu lost=%lu dup=%lu relayed=%lu PDR=%.1f%%\n",
                  s.nodeId, s.recv, s.lost, s.dup, s.relayed, pdr);
  }
}

// =================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n===== WSN GATEWAY (GATT Server) =====");

  gRxQueue = xQueueCreate(16, sizeof(RxItem));

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(9);  // cong suat phat toi da (dBm)
  // Thu tu bat buoc: dat dia chi random truoc, roi moi chuyen sang dung loai dia chi random
  bool addrOk = NimBLEDevice::setOwnAddr(NimBLEAddress(GATEWAY_STATIC_ADDR, BLE_ADDR_RANDOM)) &&
                NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);
  Serial.printf("[INIT] Dia chi BLE: %s (%s)\n", NimBLEDevice::getAddress().toString().c_str(),
                addrOk ? "random static" : "LOI - dang dung dia chi phan cung");

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new GatewayServerCallbacks());

  NimBLEService *pService = pServer->createService(GATEWAY_SERVICE_UUID);
  NimBLECharacteristic *pRxData = pService->createCharacteristic(
      GATEWAY_RXDATA_UUID,
      NIMBLE_PROPERTY::WRITE);  // chi WRITE (co response = ACK), khong mo WRITE_NR
  pRxData->setCallbacks(new RxDataCallbacks());

  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  pAdv->enableScanResponse(true);         // ten dat trong scan response (goi quang cao chinh
  pAdv->setName(DEVICE_NAME);             // chi 31 byte, UUID 128-bit da chiem 18 byte)
  pAdv->addServiceUUID(GATEWAY_SERVICE_UUID);
  bool ok = pAdv->start();                // start() tu khoi dong GATT server

  Serial.printf("[INIT] Service %s\n", GATEWAY_SERVICE_UUID);
  Serial.printf("[INIT] RxData  %s (goi %u byte)\n", GATEWAY_RXDATA_UUID, sizeof(Packet));
  Serial.printf("[INIT] Quang cao: %s | so ket noi toi da: %d\n",
                ok ? "OK" : "LOI", CONFIG_BT_NIMBLE_MAX_CONNECTIONS);
}

void loop() {
  RxItem item;
  while (xQueueReceive(gRxQueue, &item, 0) == pdTRUE) {
    handlePacket(item);
  }

  static uint32_t lastRssi = 0, lastSum = 0;
  uint32_t now = millis();
  if (now - lastRssi >= RSSI_PERIOD_MS) {
    lastRssi = now;
    updateRssi();
  }
  // Lenh qua Serial: 's' = in tong ket ngay
  while (Serial.available()) {
    if (Serial.read() == 's') printSummary();
  }

  // Tong ket dinh ky, nhung CHI in khi so lieu thay doi (khong co gi xay ra -> log dung yen)
  static uint32_t lastSig = 0;
  if (now - lastSum >= SUMMARY_PERIOD_MS) {
    lastSum = now;
    uint32_t sig = gBadSize + pServer->getConnectedCount();
    for (auto &s : gStats) sig += s.recv + s.lost + s.dup;
    if (sig != lastSig) {
      lastSig = sig;
      printSummary();
    }
  }
  delay(5);
}
