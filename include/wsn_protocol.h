// wsn_protocol.h — dinh nghia DUNG CHUNG cho ca 3 node (gateway, sensor, relay).
// Moi thu 2 ben phai "khop nhau" (UUID, cau truc goi tin) dat o day, de khong bao gio bi lech.
#pragma once          // chong include 2 lan trong cung 1 file .cpp
#include <stdint.h>   // uint8_t, uint16_t, int16_t ...

// ---------- UUID ----------
// Gateway: service + characteristic RxData (WRITE) de nhan goi tin
#define GATEWAY_SERVICE_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define GATEWAY_RXDATA_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Relay: service + characteristic RelayRxData (WRITE) de nhan goi tu Node 1
// (UUID khac Gateway -> Node 1 khi quet phan biet duoc dau la Gateway, dau la Relay)
#define RELAY_SERVICE_UUID     "8086d6df-5343-4b49-82db-8773a4498b84"
#define RELAY_RXDATA_UUID      "d9ccce58-8f6c-427c-abb2-3605ed951a4c"

// ---------- Tham so giao thuc ----------
#define ACK_FAIL_THRESHOLD   3    // N: mat ACK N lan lien tiep -> chuyen sang Relay
#define REVERT_AFTER_CYCLES  10   // M: gui qua Relay thanh cong M lan -> thu quay ve Gateway
#define DEFAULT_TTL          2    // so hop toi da

#define RELAY_FLAG_DIRECT    0x00
#define RELAY_FLAG_RELAYED   0x01

// ---------- Goi tin 5 truong ----------
// __attribute__((packed)): bat compiler xep cac truong sat nhau, KHONG chen byte dem,
// de sizeof(Packet) dung bang 10 byte o moi node -> ben nhan doc lai dung tung truong.
struct __attribute__((packed)) Packet {
  uint16_t nodeId;     // 2byte Node_ID goc (vd 0x0001 = Node 1) 
  uint16_t seq;        // 2byte Sequence_Number, tang 1 moi goi -> Gateway phat hien mat goi
  float    value;      // 4byte Sensor_Payload (vd nhiet do 25.37)
  uint8_t  relayFlag;  // 1byte RELAY_FLAG_DIRECT / RELAY_FLAG_RELAYED
  uint8_t  ttl;        // 1byte Time-To-Live, moi lan chuyen tiep giam 1
};
static_assert(sizeof(Packet) == 10, "Packet phai dung 10 byte");
