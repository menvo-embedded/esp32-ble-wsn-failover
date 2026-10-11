// ble_sensor_node.cpp — NODE CAM BIEN (Sensor Node / Node 1)
// Vai tro BLE: Central / GATT Client.
// Moi SEND_PERIOD_MS tao 1 Packet (10 byte) va ghi bang Write With Response (true = co ACK).
//
// THUAT TOAN CHUYEN MACH DU PHONG (failover):
//   DIRECT: ghi vao RxData cua Gateway.
//     Mat ACK ACK_FAIL_THRESHOLD lan lien tiep -> ngat, chuyen sang RELAY.
//   RELAY : quet tim Relay (RELAY_SERVICE_UUID), ghi vao RelayRxData, relayFlag = 1.
//     Thanh cong REVERT_AFTER_CYCLES lan -> thu quay ve DIRECT (tham do Gateway).
//     Mat ACK ACK_FAIL_THRESHOLD lan lien tiep -> quay ve DIRECT (Relay cung hong).
//
// LENH TEST qua Serial Monitor (go 1 ky tu, khong can Enter):
//   n = gui 1 goi ngay (che do tay)
//   a = bat/tat tu dong gui 1 goi/giay (mac dinh TAT -> log dung yen, test tung buoc)
//   b = gia lap DUT duong Sensor -> Gateway (Sensor khong ket noi/khong gui duoc truc tiep)
//   u = khoi phuc duong truc tiep
//   s = in tong ket ngay
//   h = in lai danh sach lenh
// Build + nap: pio run -e sensor -t upload --upload-port COMx
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "wsn_protocol.h"

#define DEVICE_NAME        "WSN-Sensor-1"
#define MY_NODE_ID         0x0001
#define SEND_PERIOD_MS     1000     // chu ky gui 1 goi
#define SCAN_TIME_MS       5000     // moi lan quet toi da 5 giay
#define SUMMARY_PERIOD_MS  10000    // chu ky in bang tong ket
#define BOOT_GRACE_MS      10000    // luc khoi dong: cho toi da 10 s de tim Gateway truoc khi tinh mat ACK
#define MODE_GRACE_MS      5000     // sau khi doi che do: cho 5 s de quet + ket noi, khong tinh vao chuoi mat ACK

enum Mode { MODE_DIRECT, MODE_RELAY };
const char *modeName(Mode m) { return m == MODE_DIRECT ? "DIRECT" : "RELAY"; }

// ================= Co giao tiep giua callback va loop() =================
// Callback (task BLE) chi bat co + luu dia chi; loop() moi goi connect()/writeValue()
// (goi ham BLE blocking trong callback -> deadlock, xem CLAUDE.md).
volatile bool gDoConnect = false;   // da tim thay dich (Gateway/Relay), loop() hay ket noi
volatile bool gNeedScan  = true;    // can quet lai
NimBLEAddress gTargetAddr;

volatile Mode gMode = MODE_DIRECT;
uint32_t gModeSince = 0;            // thoi diem doi che do gan nhat
bool     gBlockDirect = false;      // lenh test 'b': gia lap dut duong truc tiep
bool     gAutoSend = false;         // lenh 'a': tu dong gui moi SEND_PERIOD_MS (mac dinh: gui bang tay 'n')

NimBLEClient *pClient = nullptr;
NimBLERemoteCharacteristic *pTargetChar = nullptr;   // RxData (Gateway) hoac RelayRxData (Relay)

// ================= Thong ke =================
uint16_t gSeq = 0;
uint32_t gSent = 0, gAckOk = 0, gAckFail = 0;
uint32_t gSentDirect = 0, gSentRelay = 0;   // so goi co ACK theo tung duong
uint32_t gSwitchToRelay = 0, gSwitchToDirect = 0;
uint8_t  gAckFailStreak = 0;
uint16_t gRelayOkCount = 0;
bool     gEverConnected = false;

// ================= Callback quet =================
class SensorScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *dev) override {
    Mode m = gMode;
    if (m == MODE_DIRECT && gBlockDirect) return;   // dang gia lap dut: "khong nghe thay" Gateway
    const char *uuid = (m == MODE_DIRECT) ? GATEWAY_SERVICE_UUID : RELAY_SERVICE_UUID;
    if (!dev->isAdvertisingService(NimBLEUUID(uuid))) return;

    Serial.printf("[SCAN] Tim thay %s %s rssi=%d\n", m == MODE_DIRECT ? "Gateway" : "Relay",
                  dev->getAddress().toString().c_str(), dev->getRSSI());
    gTargetAddr = dev->getAddress();
    gDoConnect = true;
    NimBLEDevice::getScan()->stop();
  }

  void onScanEnd(const NimBLEScanResults &results, int reason) override {
    if (!gDoConnect) {
      Serial.printf("[SCAN] Khong thay %s -> quet lai\n", gMode == MODE_DIRECT ? "Gateway" : "Relay");
      gNeedScan = true;
    }
  }
};

// ================= Callback ket noi =================
class SensorClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *c) override {
    Serial.printf("[CONN] Da ket noi %s\n", c->getPeerAddress().toString().c_str());
  }
  void onConnectFail(NimBLEClient *c, int reason) override {
    Serial.printf("[CONN] !! Ket noi that bai, reason=0x%02X\n", reason);
    gNeedScan = true;
  }
  void onDisconnect(NimBLEClient *c, int reason) override {
    Serial.printf("[CONN] - Mat ket noi, reason=0x%02X\n", reason);
    pTargetChar = nullptr;
    gNeedScan = true;
  }
};

// -----------------------------------------------------------------
bool isReady() {
  return pClient != nullptr && pClient->isConnected() && pTargetChar != nullptr;
}

void startScan() {
  gNeedScan = false;
  gDoConnect = false;
  Serial.printf("[SCAN] Dang quet tim %s...\n", gMode == MODE_DIRECT ? "Gateway" : "Relay");
  NimBLEDevice::getScan()->start(SCAN_TIME_MS, false, true);
}

bool connectToTarget() {
  if (pClient == nullptr) {
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new SensorClientCallbacks(), false);
    // Interval 50-100 ms (don vi 1.25 ms), latency 0, supervision timeout 2 s (don vi 10 ms)
    pClient->setConnectionParams(40, 80, 0, 200);
    pClient->setConnectTimeout(5000);
  }

  Mode m = gMode;
  const char *svcUuid  = (m == MODE_DIRECT) ? GATEWAY_SERVICE_UUID : RELAY_SERVICE_UUID;
  const char *charUuid = (m == MODE_DIRECT) ? GATEWAY_RXDATA_UUID  : RELAY_RXDATA_UUID;

  Serial.printf("[CONN] Dang ket noi %s (%s) ...\n", gTargetAddr.toString().c_str(), modeName(m));
  if (!pClient->connect(gTargetAddr)) return false;

  NimBLERemoteService *svc = pClient->getService(svcUuid);
  pTargetChar = svc ? svc->getCharacteristic(charUuid) : nullptr;
  if (pTargetChar == nullptr || !pTargetChar->canWrite()) {
    Serial.println("[CONN] !! Khong tim thay characteristic de ghi -> ngat");
    pTargetChar = nullptr;
    pClient->disconnect();
    return false;
  }

  Serial.printf("[CONN] San sang gui (%s) | interval=%.2f ms\n", modeName(m),
                pClient->getConnInfo().getConnInterval() * 1.25f);
  gEverConnected = true;
  return true;
}

// Doi che do: ngat ket noi hien tai, dung quet, quet lai dich moi
void setMode(Mode m, const char *why) {
  Serial.printf("\n[FAILOVER] ===== %s -> %s : %s =====\n\n", modeName(gMode), modeName(m), why);
  if (m == MODE_RELAY) gSwitchToRelay++; else gSwitchToDirect++;
  gMode = m;
  gModeSince = millis();
  gAckFailStreak = 0;
  gRelayOkCount = 0;
  gDoConnect = false;
  if (NimBLEDevice::getScan()->isScanning()) NimBLEDevice::getScan()->stop();
  pTargetChar = nullptr;
  // disconnect() la BAT DONG BO: loop() se cho clientIdle() roi moi quet/ket noi lai
  // (neu ket noi ngay -> "Client not disconnected, cannot connect")
  if (pClient != nullptr && pClient->isConnected()) pClient->disconnect();
  gNeedScan = true;
}

// Ket qua 1 lan gui
enum SendResult { SEND_OK, SEND_FAIL, SEND_WAIT };

SendResult sendPacket() {
  Mode m = gMode;
  Packet pkt;
  pkt.nodeId    = MY_NODE_ID;
  pkt.seq       = gSeq;
  pkt.value     = 25.0f + (esp_random() % 100) / 100.0f;   // gia lap nhiet do 25.00 - 25.99
  pkt.relayFlag = (m == MODE_RELAY) ? RELAY_FLAG_RELAYED : RELAY_FLAG_DIRECT;
  pkt.ttl       = DEFAULT_TTL;

  gSeq++;    // tang ca khi gui loi -> Gateway thay khoang trong seq = goi bi mat
  gSent++;

  // Gia lap dut duong truc tiep: goi gui thang coi nhu khong co ACK
  if (m == MODE_DIRECT && gBlockDirect) {
    if (pClient != nullptr && pClient->isConnected()) pClient->disconnect();
    Serial.printf("[TX] seq=%u (DIRECT) -> ACK FAIL (gia lap dut duong)\n", pkt.seq);
    return SEND_FAIL;
  }

  if (!isReady()) {
    // Vua doi che do / vua khoi dong: dang quet + ket noi, chua tinh vao chuoi mat ACK
    bool inGrace = (millis() - gModeSince < MODE_GRACE_MS);
    Serial.printf("[TX] seq=%u (%s) KHONG GUI: chua ket noi%s\n", pkt.seq, modeName(m),
                  inGrace ? " (dang cho ket noi)" : "");
    return inGrace ? SEND_WAIT : SEND_FAIL;
  }

  uint32_t t0 = millis();
  bool ok = pTargetChar->writeValue((uint8_t *)&pkt, sizeof(pkt), true);  // true = With Response
  uint32_t rtt = millis() - t0;

  int rssi = pClient->getRssi();   // an toan: dang o loop(), khong phai callback
  Serial.printf("[TX] seq=%u (%s) value=%.2f rssi=%d -> %s (%lu ms)\n", pkt.seq, modeName(m),
                pkt.value, rssi, ok ? "ACK OK" : "ACK FAIL", rtt);
  return ok ? SEND_OK : SEND_FAIL;
}

void handleResult(SendResult r) {
  if (r == SEND_OK) {
    gAckOk++;
    gAckFailStreak = 0;
    if (gMode == MODE_DIRECT) {
      gSentDirect++;
    } else {
      gSentRelay++;
      gRelayOkCount++;
      if (gRelayOkCount >= REVERT_AFTER_CYCLES) {
        setMode(MODE_DIRECT, "da gui qua Relay du M lan, tham do lai Gateway");
      }
    }
    return;
  }

  gAckFail++;
  if (r == SEND_WAIT) return;   // khong tinh vao chuoi

  gAckFailStreak++;
  Serial.printf("[ACK] Mat ACK lien tiep: %u/%d\n", gAckFailStreak, ACK_FAIL_THRESHOLD);
  if (gAckFailStreak >= ACK_FAIL_THRESHOLD) {
    if (gMode == MODE_DIRECT) {
      setMode(MODE_RELAY, "mat ACK N lan lien tiep toi Gateway");
    } else {
      setMode(MODE_DIRECT, "Relay cung mat ACK N lan, thu lai Gateway");
    }
  }
}

void printSummary() {
  float ackRate = gSent ? 100.0f * gAckOk / gSent : 0;
  Serial.printf("[SUM] t=%lus mode=%s ket_noi=%s gia_lap_dut=%s | sent=%lu ack_ok=%lu "
                "(direct=%lu relay=%lu) ack_fail=%lu ACK=%.1f%% | chuyen->relay=%lu chuyen->direct=%lu\n",
                millis() / 1000, modeName(gMode), isReady() ? "CO" : "KHONG",
                gBlockDirect ? "CO" : "KHONG", gSent, gAckOk, gSentDirect, gSentRelay,
                gAckFail, ackRate, gSwitchToRelay, gSwitchToDirect);
}

void printHelp() {
  Serial.println("[HELP] n = gui 1 goi | a = bat/tat tu dong gui 1 goi/giay | b = gia lap dut duong truc tiep");
  Serial.println("[HELP] u = khoi phuc duong truc tiep | s = tong ket | h = danh sach lenh");
  Serial.printf("[HELP] Hien tai: mode=%s ket_noi=%s tu_dong=%s gia_lap_dut=%s\n", modeName(gMode),
                isReady() ? "CO" : "KHONG", gAutoSend ? "BAT" : "TAT", gBlockDirect ? "CO" : "KHONG");
}

void handleSerialCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'n') {
      Serial.println("[CMD] n: gui 1 goi");
      handleResult(sendPacket());
    } else if (c == 'a') {
      gAutoSend = !gAutoSend;
      Serial.printf("\n[CMD] a: tu dong gui %s\n\n", gAutoSend ? "BAT (1 goi/giay)" : "TAT (go n de gui tay)");
    } else if (c == 'h') {
      printHelp();
    } else if (c == 'b') {
      gBlockDirect = true;
      Serial.println("\n[TEST] >>> GIA LAP DUT duong Sensor -> Gateway <<<\n");
    } else if (c == 'u') {
      gBlockDirect = false;
      Serial.println("\n[TEST] >>> KHOI PHUC duong Sensor -> Gateway <<<\n");
    } else if (c == 's') {
      printSummary();
    }
  }
}

// =================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n===== WSN SENSOR NODE (GATT Client + Failover) =====");
  Serial.printf("[INIT] Node_ID=0x%04X | gui moi %d ms | N=%d M=%d\n",
                MY_NODE_ID, SEND_PERIOD_MS, ACK_FAIL_THRESHOLD, REVERT_AFTER_CYCLES);

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(9);
  Serial.printf("[INIT] Dia chi BLE: %s\n", NimBLEDevice::getAddress().toString().c_str());

  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new SensorScanCallbacks(), false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(100);

  gModeSince = millis();
  printHelp();
}

void loop() {
  handleSerialCommand();

  // Client con dang giu ket noi cu (vua goi disconnect(), chua ngat xong) -> chua quet/ket noi.
  // KHONG dung !isConnected(): disconnect() dat trang thai DISCONNECTING ngay -> isConnected() = false
  // du chua ngat xong, ma connect() lai tu choi khi trang thai != DISCONNECTED.
  // Handle ket noi chi ve BLE_HS_CONN_HANDLE_NONE khi da ngat XONG -> dung handle de kiem tra.
  bool clientIdle = (pClient == nullptr || pClient->getConnHandle() == BLE_HS_CONN_HANDLE_NONE);

  // 1) Tim thay dich -> ket noi
  if (gDoConnect && clientIdle) {
    gDoConnect = false;
    if (!connectToTarget()) gNeedScan = true;
  }

  // 2) Can quet lai
  if (gNeedScan && clientIdle && !NimBLEDevice::getScan()->isScanning()) {
    startScan();
  }

  uint32_t now = millis();

  // 3) Che do tu dong: gui dinh ky (bat dau sau lan ket noi dau tien, hoac sau BOOT_GRACE_MS)
  static uint32_t lastSend = 0;
  if (gAutoSend && (gEverConnected || now > BOOT_GRACE_MS) && now - lastSend >= SEND_PERIOD_MS) {
    lastSend = now;
    handleResult(sendPacket());
  }

  // 4) Tong ket dinh ky (chi khi tu dong gui; che do tay thi go 's')
  static uint32_t lastSum = 0;
  if (gAutoSend && now - lastSum >= SUMMARY_PERIOD_MS) {
    lastSum = now;
    printSummary();
  }

  delay(5);
}
