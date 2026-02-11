# Aerobot

**Data Formats**
- UNO to ESP32 serial line
  `UNO,1,<loco>,<gas>,<mq2>,<mq7>,<mq135>,<gasRaw>,<gasRise>,<pir>,<dist>,<ldr>`
- ESP32 to UNO serial line
  `FIRE32,confidence=<value>`
- ESP32 to server feed
  `GET /feed?data=DATA,<fire_conf>,<fire_state>,<loco>,<gas>,<mq2>,<mq7>,<mq135>,<gasRaw>,<gasRise>,<pir>,<dist>,<ldr>,<heap>`

**Server Endpoints**
- `GET /ui` live dashboard
- `GET /api/latest` latest parsed telemetry
- `GET /api/history?limit=120` telemetry history
- `GET /api/fireframe` latest fire snapshot
- `GET /api/fireframe-meta` metadata for latest fire snapshot
- `GET /recent` recent event log
- `GET /status` server status
- `POST /fireframe` multipart upload from ESP32 (`confidence`, `frame`)
- `GET /feed` telemetry ingest
- `GET /fireevent` optional confidence event
- `GET /selftest` connectivity check

**Quick Start**
1. Start the server.
2. Flash the Arduino UNO firmware.
3. Flash the ESP32 firmware and confirm Wi-Fi connects (or AP fallback).
4. Power the robot and open the server UI.

**Server**
1. `cd aerobot-server`
2. `npm install` (optional if `node_modules` already present)
3. `node index.js`
4. Open `http://<server-ip>:3000/ui`

**ESP32 Firmware Configuration**
- Edit `WIFI_SSID`, `WIFI_PASS`, and `SERVER_HOST` in `aerobot-esp32/aerobot-esp32.ino`.
- The device attempts STA for about 8 seconds, then falls back to AP `ESP32-RX` with password `esp32rx1`.
- Camera settings and fire thresholds are in the same file.
