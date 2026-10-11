# ESP32 BLE Wireless Sensor Network with Failover

An academic wireless sensor network prototype built with three ESP32 DevKit V1 boards and Bluetooth Low Energy (BLE). The system uses a custom GATT-based relay path to maintain data delivery when the direct Sensor-to-Gateway link becomes unavailable.

## Architecture

```text
Normal path:    Sensor  ──────────────> Gateway
Failover path:  Sensor  ──> Relay ────> Gateway
```

- **Gateway:** BLE Peripheral / GATT Server that receives sensor packets.
- **Sensor:** BLE Central / GATT Client that scans, connects, sends packets, and monitors acknowledgements.
- **Relay:** Dual-role node. It receives packets as a GATT Server and forwards them to the Gateway as a GATT Client.

The project uses BLE only; Wi-Fi and Bluetooth Mesh are not used.

## Key Features

- Custom 10-byte binary packet shared by all three nodes:

  ```text
  nodeId (2 B) | sequence (2 B) | sensor value (4 B) | relay flag (1 B) | TTL (1 B)
  ```

- GATT Write With Response for hop-by-hop acknowledgement.
- Sensor failover after three consecutive missing acknowledgements.
- Automatic direct-path retry after ten successful transmissions through the Relay.
- Sequence-based packet-loss and duplicate detection at the Gateway.
- TTL-based loop prevention and direct/relayed packet identification.
- FreeRTOS queues keep blocking BLE work out of callbacks.
- RSSI, Packet Delivery Ratio (PDR), and CSV-formatted Serial logs for testing.

## Repository Structure

```text
include/wsn_protocol.h       Shared UUIDs, protocol constants, and packet format
src/ble_gateway_node.cpp     Gateway firmware
src/ble_sensor_node.cpp      Sensor firmware and failover state machine
src/ble_relay_node.cpp       Dual-role Relay firmware
luu-do/                      Flowcharts and editable Draw.io source
platformio.ini               PlatformIO build environments
```

## Build

Requirements:

- VS Code with PlatformIO, or PlatformIO Core
- ESP32 DevKit V1 boards
- NimBLE-Arduino 2.5.1

Build each node independently:

```bash
pio run -e gateway
pio run -e sensor
pio run -e relay
```

Upload and monitor a board:

```bash
pio run -e gateway -t upload --upload-port COMx
pio device monitor -p COMx -b 115200
```

Replace `gateway` with `sensor` or `relay` for the other boards.

## Tested Scenario

The prototype was exercised on three physical ESP32 boards:

1. The Sensor discovered and connected directly to the Gateway.
2. Packets were delivered using Write With Response acknowledgements.
3. A direct-link failure was simulated.
4. After three missing acknowledgements, the Sensor switched to the Relay.
5. The Relay forwarded packets to the Gateway and decremented TTL.
6. After ten successful relayed transmissions, the Sensor retried the direct path.

## Known Limitations

- Acknowledgements are hop-by-hop. In relay mode, the Sensor knows that the Relay received a packet, not that the Gateway received it end-to-end.
- Sensor values are currently simulated; a physical sensing device can replace the generated value.
- Power consumption and sleep-mode measurements are planned work, not implemented results.
- The current prototype uses one Sensor and one Relay; larger-network routing is outside the present scope.

## Development Note

This project used AI-assisted coding during initial firmware development. My work focused on defining system behavior, testing the prototype on physical ESP32 boards, analyzing Serial logs, reproducing failures, and studying the Gateway-Sensor firmware flow. The repository documents the tested prototype and the technical concepts I can explain and continue developing.
