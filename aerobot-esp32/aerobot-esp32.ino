#include "esp_camera.h"
#include <Arduino.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ================= UART ================= */
static const int UART_RX_PIN = 14;
static const int UART_TX_PIN = 15;
static HardwareSerial Link(2);

/* ================= LEDs ================= */
#define LED_FLASH_PIN 4
#define LED_RED_PIN 33

static const uint32_t LED_PWM_FREQ = 5000;
static const uint8_t LED_PWM_RES = 8;
static const uint8_t LED_ON_LEVEL = 40;
static const uint8_t LED_DETECT_LEVEL = 18;

static const uint32_t RED_BLINK_MS = 120;
static uint32_t redOffAt = 0;

static bool flashOn = false;
static bool sentForThisFire = false;
static const int LDR_FLASH_ON = 800;
static const int LDR_FLASH_OFF = 750;

/* ================= WIFI ================= */
const char *WIFI_SSID = "Gavesh";
const char *WIFI_PASS = "123123123";
const char *AP_SSID = "ESP32-RX";
const char *AP_PASS = "esp32rx1";

static WebServer server(80);

/* ================= SERVER ================= */
const bool ENABLE_SERVER_PUSH = true;
const char *SERVER_HOST = "172.20.10.4";
const uint16_t SERVER_PORT = 3000;
const char *SERVER_FEED_PATH = "/feed";
const uint32_t FEED_INTERVAL_MS = 1000;

const float FIRE_ON_TH = 0.65f;
const float FIRE_OFF_TH = 0.55f;

/* ================= RX BUFFER + LOG ================= */
#define LOG_LINES 20
#define MAX_LINE 240

static char logLines[LOG_LINES][MAX_LINE];
static int logHead = 0;
static int logCount = 0;
static char rxBuf[MAX_LINE];
static int rxLen = 0;
static char lastLine[MAX_LINE];
static uint32_t lastRxMs = 0;

/* ================= LIVE DATA ================= */
struct LiveData {
  bool valid = false;
  int version = 0;
  int locoState = 0;
  int gasState = 0;
  int mq2 = 0, mq7 = 0, mq135 = 0;
  int gasRaw = 0;
  int gasRise = 0;
  int pir = 0;
  int dist = -1;
  int ldr = 0;
  uint32_t lastParseMs = 0;
};

static LiveData live;
static bool doParse = true;

/* ================= FIRE ================= */
static float lastFirePercent = 0.0f;
static uint32_t lastFireMs = 0;
static const uint32_t FIRE_CHECK_INTERVAL_MS = 900;
static uint32_t lastFeedMs = 0;
static uint32_t lastSentParseMs = 0;

static float prevFireConf = 0.0f;
static float fireFlickEma = 0.0f;
static bool fireStateHys = false;

/* ================= CAMERA PINS ================= */
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

/* ================= UTIL ================= */
static inline void setFlash(uint8_t v) {
  ledcWrite(LED_FLASH_PIN, v);
}

static inline void setRed(bool on) {
  digitalWrite(LED_RED_PIN, on ? HIGH : LOW);
}

static void pushLine(const char *s) {
  strncpy(logLines[logHead], s, MAX_LINE - 1);
  logLines[logHead][MAX_LINE - 1] = 0;
  logHead = (logHead + 1) % LOG_LINES;
  if (logCount < LOG_LINES)
    logCount++;
  strncpy(lastLine, s, MAX_LINE - 1);
  lastLine[MAX_LINE - 1] = 0;
  lastRxMs = millis();
}

/* ================= PARSER =================
   UNO,1,<loco>,<gas>,<mq2>,<mq7>,<mq135>,<gasRaw>,<gasRise>,<pir>,<dist>,<ldr>
*/
static bool parseLiveLine(const char *line) {
  if (strncmp(line, "UNO,", 4) != 0)
    return false;

  double vals[12];
  int n = 0;

  const char *p = line + 4;
  while (*p && n < 12) {
    char *e;
    double v = strtod(p, &e);
    if (e == p)
      break;
    vals[n++] = v;
    p = (*e == ',') ? (e + 1) : e;
  }

  if (n < 11)
    return false;

  live.version = (int)vals[0];
  live.locoState = (int)vals[1];
  live.gasState = (int)vals[2];
  live.mq2 = (int)vals[3];
  live.mq7 = (int)vals[4];
  live.mq135 = (int)vals[5];
  live.gasRaw = (int)vals[6];
  live.gasRise = (int)vals[7];
  live.pir = (int)vals[8];
  live.dist = (int)vals[9];
  live.ldr = (int)vals[10];

  live.lastParseMs = millis();
  live.valid = true;
  return true;
}

static void sendFeedIfReady() {
  if (!ENABLE_SERVER_PUSH)
    return;
  if (!SERVER_HOST || !SERVER_HOST[0])
    return;
  if (WiFi.status() != WL_CONNECTED)
    return;
  if (!live.valid)
    return;

  uint32_t now = millis();
  if (now - lastFeedMs < FEED_INTERVAL_MS)
    return;
  if (live.lastParseMs == lastSentParseMs)
    return;

  lastFeedMs = now;
  lastSentParseMs = live.lastParseMs;

  float fireConf = lastFirePercent / 100.0f;
  int fireState = fireStateHys ? 1 : 0;

  char data[200];
  snprintf(data, sizeof(data), "DATA,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
           fireConf, fireState, live.locoState, live.gasState, live.mq2,
           live.mq7, live.mq135, live.gasRaw, live.gasRise, live.pir, live.dist,
           live.ldr, (int)ESP.getFreeHeap());

  char url[320];
  snprintf(url, sizeof(url), "http://%s:%u%s?data=%s", SERVER_HOST, SERVER_PORT,
           SERVER_FEED_PATH, data);

  WiFiClient client;
  HTTPClient http;
  http.setTimeout(600);
  if (http.begin(client, url)) {
    int code = http.GET();
    Serial.print("FEED ");
    Serial.print(code);
    Serial.print(" ");
    Serial.println(url);
    http.end();
  }
}

/* ================= FIRE DETECTION ================= */
static inline void rgb565_to_rgb888(uint16_t p, uint8_t &r, uint8_t &g,
                                    uint8_t &b) {
  r = ((p >> 11) & 0x1F) << 3;
  g = ((p >> 5) & 0x3F) << 2;
  b = (p & 0x1F) << 3;
}

static bool uploadFireFrame(float conf01) {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (!SERVER_HOST || !SERVER_HOST[0]) return false;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;

  uint8_t *jpgBuf = nullptr;
  size_t jpgLen = 0;
  bool needsFree = false;

  if (fb->format == PIXFORMAT_JPEG) {
    jpgBuf = fb->buf;
    jpgLen = fb->len;
  } else {
    if (!frame2jpg(fb, 65, &jpgBuf, &jpgLen) || !jpgBuf || jpgLen == 0) {
      esp_camera_fb_return(fb);
      return false;
    }
    needsFree = true;
  }

  const char *boundary = "----AerobotBoundary7MA4YWxk";

  char head[512];
  int headLen = snprintf(
    head, sizeof(head),
    "--%s\r\n"
    "Content-Disposition: form-data; name=\"confidence\"\r\n\r\n"
    "%.3f\r\n"
    "--%s\r\n"
    "Content-Disposition: form-data; name=\"frame\"; filename=\"fire.jpg\"\r\n"
    "Content-Type: image/jpeg\r\n\r\n",
    boundary, conf01, boundary);

  char tail[64];
  int tailLen = snprintf(tail, sizeof(tail), "\r\n--%s--\r\n", boundary);

  size_t totalLen = (size_t)headLen + jpgLen + (size_t)tailLen;

  WiFiClient client;
  client.setTimeout(2500);

  if (!client.connect(SERVER_HOST, SERVER_PORT)) {
    if (needsFree) free(jpgBuf);
    esp_camera_fb_return(fb);
    return false;
  }

  client.print("POST /fireframe HTTP/1.1\r\n");
  client.print("Host: ");
  client.print(SERVER_HOST);
  client.print("\r\n");
  client.print("Connection: close\r\n");
  client.print("Content-Type: multipart/form-data; boundary=");
  client.print(boundary);
  client.print("\r\n");
  client.print("Content-Length: ");
  client.print((uint32_t)totalLen);
  client.print("\r\n\r\n");

  client.write((const uint8_t *)head, headLen);
  client.write(jpgBuf, jpgLen);
  client.write((const uint8_t *)tail, tailLen);

  uint32_t t0 = millis();
  while (!client.available() && (millis() - t0) < 2500) delay(1);

  int code = -1;
  String statusLine = client.readStringUntil('\n');  // e.g. "HTTP/1.1 200 OK"
  if (statusLine.startsWith("HTTP/1.1 ")) code = statusLine.substring(9, 12).toInt();
  else if (statusLine.startsWith("HTTP/1.0 ")) code = statusLine.substring(9, 12).toInt();

  client.stop();

  if (needsFree) free(jpgBuf);
  esp_camera_fb_return(fb);

  Serial.print("FIRE SNAP HTTP: ");
  Serial.println(code);

  return (code >= 200 && code < 300);
}

static float detectFireConfidenceCore() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
    return 0.0f;

  if (fb->format != PIXFORMAT_RGB565 || !fb->buf) {
    esp_camera_fb_return(fb);
    return 0.0f;
  }

  const uint16_t *px = (const uint16_t *)fb->buf;
  int w = fb->width, h = fb->height;

  int fire = 0, strong = 0, total = 0;

  for (int y = h / 4; y < 3 * h / 4; y += 4) {
    int row = y * w;
    for (int x = w / 4; x < 3 * w / 4; x += 4) {
      uint8_t r, g, b;
      rgb565_to_rgb888(px[row + x], r, g, b);

      if (r > 235 && g > 235 && b > 235) {
        total++;
        continue;
      }

      int Y = (77 * (int)r + 150 * (int)g + 29 * (int)b) >> 8;

      if (Y > 70 && r > 190 && g > 80 && b < 120 && r > (uint8_t)(g + 30)) {
        fire++;
        if (r > 220 && g > 120)
          strong++;
      }

      total++;
    }
  }

  esp_camera_fb_return(fb);

  if (fire < 5 || total <= 0)
    return 0.0f;

  float area = (float)fire / (float)total;
  float strength = (fire > 0) ? ((float)strong / (float)fire) : 0.0f;

  float conf = area * 10.0f + strength * 0.25f;
  if (conf > 1.0f)
    conf = 1.0f;
  return conf;
}

static float detectFireConfidence() {
  float conf = detectFireConfidenceCore();

  float d = conf - prevFireConf;
  if (d < 0)
    d = -d;
  fireFlickEma = fireFlickEma * 0.85f + d * 0.15f;
  prevFireConf = conf;

  if (fireFlickEma < 0.03f)
    conf *= 0.35f;
  else if (fireFlickEma > 0.08f)
    conf *= 1.15f;

  if (conf > 1.0f)
    conf = 1.0f;
  return conf;
}

static float detectFireConfidenceWithProbeFlash(bool dark) {
  float conf1 = detectFireConfidence();

  float conf = conf1;
  bool near = (conf1 > 0.35f && conf1 < 0.75f);

  if (dark && near) {
    setFlash(LED_DETECT_LEVEL);
    delay(35);
    float conf2 = detectFireConfidence();
    setFlash(0);
    if (conf2 > conf)
      conf = conf2;
  }

  return conf;
}

/* ================= CAMERA INIT ================= */
static void init_camera() {
  camera_config_t c{};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_RGB565;
  c.frame_size = FRAMESIZE_QQVGA;
  c.jpeg_quality = 12;
  c.fb_count = 1;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;

  esp_camera_init(&c);

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_saturation(s, 1);
    s->set_contrast(s, 1);
  }
}

/* ================= WEB ================= */
static void handleRoot() {
  IPAddress ip = (WiFi.getMode() == WIFI_AP) ? WiFi.softAPIP() : WiFi.localIP();

  char page[1800];
  snprintf(page, sizeof(page),
           "<!doctype html><html><head><meta charset='utf-8'>"
           "<meta name='viewport' content='width=device-width,initial-scale=1'>"
           "<title>ESP32 LIVE</title>"
           "<style>"
           "body{margin:0;font-family:ui-monospace,Consolas,monospace;"
           "background:#0b0f14;color:#e6edf3;padding:16px;}"
           ".card{background:#121925;border:1px solid "
           "#1f2a3a;border-radius:10px;padding:12px;max-width:900px;}"
           "pre{white-space:pre-wrap;word-break:break-word;margin:0;}"
           ".muted{color:#9fb1c1;font-size:12px;margin:0 0 10px 0;}"
           "</style></head><body>"
           "<div class='card'>"
           "<div class='muted'>IP: %u.%u.%u.%u · updates via /status</div>"
           "<pre id='out'>loading...</pre>"
           "</div>"
           "<script>"
           "async function tick(){"
           " try{"
           "  const r=await fetch('/status',{cache:'no-store'});"
           "  const t=await r.text();"
           "  document.getElementById('out').textContent=t;"
           " }catch(e){"
           "  document.getElementById('out').textContent='(disconnected)';"
           " }"
           "}"
           "setInterval(tick,600);"
           "tick();"
           "</script>"
           "</body></html>",
           ip[0], ip[1], ip[2], ip[3]);

  server.send(200, "text/html", page);
}

static void handleStatus() {
  char out[820];

  uint32_t now = millis();
  uint32_t rxAgo = (lastRxMs == 0) ? 0 : (now - lastRxMs) / 1000;
  uint32_t fireAgo = (lastFireMs == 0) ? 0 : (now - lastFireMs) / 1000;

  if (!live.valid) {
    snprintf(out, sizeof(out),
             "Last Received Line: %s\n"
             "Parsed: (none)\n"
             "Last RX: %s\n"
             "Fire: %.1f%% (%lus ago)\n"
             "FireState: %d\n"
             "FlickEma: %.3f\n"
             "Heap: %u",
             lastLine[0] ? lastLine : "(none)", lastRxMs ? "" : "never",
             lastFirePercent, (unsigned long)fireAgo, fireStateHys ? 1 : 0,
             fireFlickEma, (unsigned)ESP.getFreeHeap());
    if (lastRxMs) {
      char tmp[64];
      snprintf(tmp, sizeof(tmp), "%lus ago", (unsigned long)rxAgo);
      char out2[820];
      snprintf(out2, sizeof(out2),
               "Last Received Line: %s\n"
               "Parsed: (none)\n"
               "Last RX: %s\n"
               "Fire: %.1f%% (%lus ago)\n"
               "FireState: %d\n"
               "FlickEma: %.3f\n"
               "Heap: %u",
               lastLine[0] ? lastLine : "(none)", tmp, lastFirePercent,
               (unsigned long)fireAgo, fireStateHys ? 1 : 0, fireFlickEma,
               (unsigned)ESP.getFreeHeap());
      server.send(200, "text/plain", out2);
      return;
    }
    server.send(200, "text/plain", out);
    return;
  }

  snprintf(out, sizeof(out),
           "Last line: %s\n\n"
           "v=%d  loco=%d  gas=%d\n"
           "mq2=%d  mq7=%d  mq135=%d\n"
           "gasRaw=%d  rise=%d  pir=%d\n"
           "dist=%d  ldr=%d\n\n"
           "Last RX: %lus ago\n"
           "Fire: %.1f%% (%lus ago)\n"
           "FireState: %d\n"
           "FlickEma: %.3f\n"
           "Flash: %s\n"
           "Heap: %u",
           lastLine[0] ? lastLine : "(none)", live.version, live.locoState,
           live.gasState, live.mq2, live.mq7, live.mq135, live.gasRaw,
           live.gasRise, live.pir, live.dist, live.ldr, (unsigned long)rxAgo,
           lastFirePercent, (unsigned long)fireAgo, fireStateHys ? 1 : 0,
           fireFlickEma, flashOn ? "ON" : "OFF", (unsigned)ESP.getFreeHeap());

  server.send(200, "text/plain", out);
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(57600);
  Link.begin(57600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  ledcAttach(LED_FLASH_PIN, LED_PWM_FREQ, LED_PWM_RES);
  setFlash(0);

  pinMode(LED_RED_PIN, OUTPUT);
  setRed(false);

  init_camera();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
    delay(50);
    yield();
  }
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
  }

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.begin();

  rxBuf[0] = 0;
  lastLine[0] = 0;

  Serial.println("ESP32 READY");
}

/* ================= LOOP ================= */
void loop() {
  server.handleClient();

  bool gotLine = false;

  while (Link.available()) {
    char c = (char)Link.read();

    if (c == '\n') {
      rxBuf[rxLen] = 0;
      pushLine(rxBuf);
      if (doParse)
        parseLiveLine(rxBuf);
      rxLen = 0;
      gotLine = true;
    } else if (c != '\r') {
      if (rxLen < MAX_LINE - 1)
        rxBuf[rxLen++] = c;
      else
        rxLen = 0;
    }
  }

  if (gotLine) {
    setRed(true);
    redOffAt = millis() + RED_BLINK_MS;
  }
  if (redOffAt && millis() > redOffAt) {
    setRed(false);
    redOffAt = 0;
  }

  sendFeedIfReady();

  if (live.valid) {
    if (!flashOn && live.ldr > LDR_FLASH_ON)
      flashOn = true;
    if (flashOn && live.ldr < LDR_FLASH_OFF)
      flashOn = false;
    setFlash(flashOn ? LED_ON_LEVEL : 0);
  } else {
    setFlash(0);
    flashOn = false;
  }

  static uint32_t lastFireCheck = 0;
  uint32_t now = millis();
  if (now - lastFireCheck >= FIRE_CHECK_INTERVAL_MS) {
    lastFireCheck = now;

    bool dark = live.valid && (live.ldr > LDR_FLASH_ON);

    uint8_t saved = flashOn ? LED_ON_LEVEL : 0;
    if (saved)
      setFlash(0);

    float conf = detectFireConfidenceWithProbeFlash(dark);
    if (conf < 0.0f)
      conf = 0.0f;
    if (conf > 1.0f)
      conf = 1.0f;

    if (saved)
      setFlash(saved);

    if (!fireStateHys && conf >= FIRE_ON_TH)
      fireStateHys = true;
    if (fireStateHys && conf <= FIRE_OFF_TH)
      fireStateHys = false;

    lastFirePercent = conf * 100.0f;
    lastFireMs = now;

    Link.print("FIRE32,confidence=");
    Link.println(lastFirePercent, 1);

    if (fireStateHys && !sentForThisFire && live.locoState == 0) {
      bool ok = uploadFireFrame(conf);
      Serial.print("uploadFireFrame: ");
      Serial.println(ok ? "OK" : "FAIL");
      sentForThisFire = ok ? true : false;
    }
    if (!fireStateHys) sentForThisFire = false;

    Serial.print("TX FIRE32,confidence=");
    Serial.println(lastFirePercent, 1);
  }

  yield();
}