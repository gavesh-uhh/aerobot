#include "esp_camera.h"
#include <Arduino.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static const int UART_RX_PIN = 14;
static const int UART_TX_PIN = 15;
static HardwareSerial Link(2);

static const int LED_FLASH_PIN = 4;
static const int LED_RED_PIN = 33;

static const uint32_t LED_PWM_FREQ = 5000;
static const uint8_t LED_PWM_RES = 8;
static const uint8_t LED_ON_LEVEL = 40;
static const uint8_t LED_DETECT_LEVEL = 18;

static const uint32_t RED_BLINK_MS = 120;

static const int LDR_FLASH_ON = 850;
static const int LDR_FLASH_OFF = 780;

static const char *WIFI_SSID = "Gavesh";
static const char *WIFI_PASS = "123123123";
static const char *AP_SSID = "Aerobot AP";
static const char *AP_PASS = "esp32rx1";

static WebServer server(80);

static const bool ENABLE_SERVER_PUSH = true;
static const char *SERVER_HOST = "172.20.10.4";
static const uint16_t SERVER_PORT = 3000;
static const char *SERVER_FEED_PATH = "/feed";
static const uint32_t FEED_INTERVAL_MS = 500;
static const uint32_t FIRE_CHECK_INTERVAL_MS = 500;

static const int GAS_RISE_ASSIST_TH = 18;

static const float FIRE_ON_TH_ASSIST = 0.45f;
static const float FIRE_OFF_TH_ASSIST = 0.35f;
static const float FIRE_ON_TH_STRONG = 0.30f;
static const float FIRE_OFF_TH_STRONG = 0.22f;

static const float FIRE_EMA_ALPHA = 0.70f;
static const uint8_t FIRE_ON_HITS = 1;
static const uint8_t FIRE_OFF_HITS = 2;

static const int FIRE_SCAN_STEP = 1;
static const int FIRE_SCAN_X0_DIV = 4;
static const int FIRE_SCAN_X1_DIV = 4;
static const int FIRE_SCAN_Y0_DIV = 4;
static const int FIRE_SCAN_Y1_DIV = 4;

static const int FIRE_WHITE_Y_MIN = 245;
static const int FIRE_WHITE_SAT_MAX = 12;

static const int FIRE_CAND_Y_MIN = 75;
static const int FIRE_CAND_SAT_MIN = 40;
static const int FIRE_CAND_R_MIN = 160;
static const int FIRE_CAND_G_MIN = 85;
static const int FIRE_CAND_B_MAX = 150;
static const int FIRE_CAND_RG_MIN = 35;
static const int FIRE_CAND_CR_MIN = 165;
static const int FIRE_CAND_CB_MAX = 135;

static const int FIRE_CORE_Y_MIN = 140;
static const int FIRE_CORE_R_MIN = 200;
static const int FIRE_CORE_CR_MIN = 168;

static const int FIRE_STRONG_Y_MIN = 168;
static const int FIRE_STRONG_R_MIN = 218;
static const int FIRE_STRONG_G_MIN = 108;
static const int FIRE_STRONG_CR_MIN = 178;

static const int FIRE_MIN_PIX = 3;
static const int FIRE_MIN_BW = 3;
static const int FIRE_MIN_BH = 3;

static const float FIRE_DENSITY_MIN = 0.05f;
static const float FIRE_CORE_RATIO_MIN = 0.03f;
static const float FIRE_AVGY_MIN = 85.0f;

static const float FIRE_RAW_INSTANT_TH = 0.90f;

static const float FIRE_W_AREA = 14.0f;
static const float FIRE_W_DENSITY = 0.35f;
static const float FIRE_W_CORE = 0.45f;
static const float FIRE_W_STRONG = 0.25f;

static const float FIRE_PROBE_C1_MIN = 0.10f;
static const float FIRE_PROBE_C1_MAX = 0.98f;

static const uint32_t FIRE_PROBE_DELAY_MS = 30;
static const float FIRE_PROBE_D_START = 0.03f;
static const float FIRE_PROBE_D_RANGE = 0.10f;
static const float FIRE_PROBE_PENALTY_STRENGTH = 0.85f;
static const float FIRE_PROBE_D_HARD = 0.08f;
static const float FIRE_PROBE_HARD_MULT = 0.35f;

static const bool RGB565_SWAP_BYTES = true;

#define MAX_LINE 240

static char rxBuf[MAX_LINE];
static int rxLen = 0;
static char lastLine[MAX_LINE];
static uint32_t lastRxMs = 0;

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

static bool flashOn = false;
static bool sentForThisFire = false;
static uint32_t redOffAt = 0;

static float lastFirePercent = 0.0f;
static uint32_t lastFireMs = 0;
static uint32_t lastFeedMs = 0;
static uint32_t lastSentParseMs = 0;

static bool fireStateHys = false;
static float fireEma = 0.0f;

static float dbg_confRaw = 0.0f;
static float dbg_confEma = 0.0f;
static float dbg_probeDelta = 0.0f;
static float dbg_density = 0.0f;
static float dbg_core = 0.0f;
static float dbg_avgY = 0.0f;
static int dbg_firePix = 0;
static int dbg_bw = 0, dbg_bh = 0;

static sensor_t *cam = nullptr;

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

static inline void setFlash(uint8_t v) { ledcWrite(LED_FLASH_PIN, v); }
static inline void setRed(bool on) { digitalWrite(LED_RED_PIN, on ? HIGH : LOW); }

static bool parseLiveLine(const char *line) {
  if (strncmp(line, "UNO,", 4) != 0) return false;

  double vals[12];
  int n = 0;

  const char *p = line + 4;
  while (*p && n < 12) {
    char *e;
    double v = strtod(p, &e);
    if (e == p) break;
    vals[n++] = v;
    p = (*e == ',') ? (e + 1) : e;
  }

  if (n < 11) return false;

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
  if (!ENABLE_SERVER_PUSH) return;
  if (!SERVER_HOST || !SERVER_HOST[0]) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (!live.valid) return;

  uint32_t now = millis();
  if (now - lastFeedMs < FEED_INTERVAL_MS) return;
  if (live.lastParseMs == lastSentParseMs) return;

  lastFeedMs = now;
  lastSentParseMs = live.lastParseMs;

  float fireConf = lastFirePercent / 100.0f;
  int fireState = fireStateHys ? 1 : 0;

  char data[240];
  snprintf(data, sizeof(data), "DATA,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
           fireConf, fireState, live.locoState, live.gasState, live.mq2,
           live.mq7, live.mq135, live.gasRaw, live.gasRise, live.pir, live.dist,
           live.ldr, (int)ESP.getFreeHeap());

  char url[380];
  snprintf(url, sizeof(url), "http://%s:%u%s?data=%s", SERVER_HOST, SERVER_PORT, SERVER_FEED_PATH, data);

  WiFiClient client;
  HTTPClient http;
  http.setTimeout(2000);
  if (http.begin(client, url)) {
    http.GET();
    http.end();
  }
}

static inline uint16_t read565(const uint16_t *p) {
  uint16_t v = *p;
  if (!RGB565_SWAP_BYTES) return v;
  return (uint16_t)((v << 8) | (v >> 8));
}

static inline void rgb565_to_rgb888(uint16_t p, uint8_t &r, uint8_t &g, uint8_t &b) {
  r = ((p >> 11) & 0x1F) << 3;
  g = ((p >> 5) & 0x3F) << 2;
  b = (p & 0x1F) << 3;
}

struct FireM {
  int fire = 0;
  int strong = 0;
  int core = 0;
  int total = 0;
  int minSx = 9999, minSy = 9999, maxSx = -1, maxSy = -1;
  int sumY = 0;
};

static float scoreFireFromFb(camera_fb_t *fb) {
  if (!fb || fb->format != PIXFORMAT_RGB565 || !fb->buf) return 0.0f;

  FireM m;

  const uint16_t *px = (const uint16_t *)fb->buf;
  int w = fb->width, h = fb->height;

  int x0 = w / FIRE_SCAN_X0_DIV;
  int x1 = w - (w / FIRE_SCAN_X1_DIV);
  int y0 = h / FIRE_SCAN_Y0_DIV;
  int y1 = h - (h / FIRE_SCAN_Y1_DIV);

  for (int y = y0; y < y1; y += FIRE_SCAN_STEP) {
    int row = y * w;
    int sy = y / FIRE_SCAN_STEP;

    for (int x = x0; x < x1; x += FIRE_SCAN_STEP) {
      int sx = x / FIRE_SCAN_STEP;

      uint8_t r, g, b;
      rgb565_to_rgb888(read565(&px[row + x]), r, g, b);

      int maxc = r;
      if (g > maxc) maxc = g;
      if (b > maxc) maxc = b;

      int minc = r;
      if (g < minc) minc = g;
      if (b < minc) minc = b;

      int sat = maxc - minc;

      int Y = (77 * (int)r + 150 * (int)g + 29 * (int)b) >> 8;
      int Cb = (((-43 * (int)r - 85 * (int)g + 128 * (int)b) >> 8) + 128);
      int Cr = (((128 * (int)r - 107 * (int)g - 21 * (int)b) >> 8) + 128);

      bool whiteish = (Y > FIRE_WHITE_Y_MIN && sat < FIRE_WHITE_SAT_MAX);
      if (whiteish) {
        m.total++;
        continue;
      }

      bool cand =
        (Y > FIRE_CAND_Y_MIN) &&
        (sat > FIRE_CAND_SAT_MIN) &&
        (r > FIRE_CAND_R_MIN) &&
        (g > FIRE_CAND_G_MIN) &&
        (b < FIRE_CAND_B_MAX) &&
        (r > g) && (g >= b) &&
        ((r - g) > FIRE_CAND_RG_MIN) &&
        (Cr > FIRE_CAND_CR_MIN) &&
        (Cb < FIRE_CAND_CB_MAX);

      if (cand) {
        m.fire++;
        m.sumY += Y;

        bool core = (Y > FIRE_CORE_Y_MIN && r > FIRE_CORE_R_MIN && Cr > FIRE_CORE_CR_MIN);
        if (core) m.core++;

        bool strong = (Y > FIRE_STRONG_Y_MIN && r > FIRE_STRONG_R_MIN && g > FIRE_STRONG_G_MIN && Cr > FIRE_STRONG_CR_MIN);
        if (strong) m.strong++;

        if (sx < m.minSx) m.minSx = sx;
        if (sx > m.maxSx) m.maxSx = sx;
        if (sy < m.minSy) m.minSy = sy;
        if (sy > m.maxSy) m.maxSy = sy;
      }

      m.total++;
    }
  }

  dbg_firePix = m.fire;

  if (m.total <= 0 || m.fire < FIRE_MIN_PIX) {
    dbg_density = 0.0f;
    dbg_core = 0.0f;
    dbg_avgY = 0.0f;
    dbg_bw = 0;
    dbg_bh = 0;
    return 0.0f;
  }

  int bw = (m.maxSx - m.minSx + 1);
  int bh = (m.maxSy - m.minSy + 1);
  dbg_bw = bw;
  dbg_bh = bh;

  if (bw < FIRE_MIN_BW || bh < FIRE_MIN_BH) return 0.0f;

  int boxArea = bw * bh;
  float density = (float)m.fire / (float)boxArea;
  float area = (float)m.fire / (float)m.total;
  float coreR = (float)m.core / (float)m.fire;
  float avgY = (float)m.sumY / (float)m.fire;
  float strongR = (float)m.strong / (float)m.fire;

  dbg_density = density;
  dbg_core = coreR;
  dbg_avgY = avgY;

  int roiW = (x1 - x0) / FIRE_SCAN_STEP;
  int roiH = (y1 - y0) / FIRE_SCAN_STEP;
  if (bw > (int)(roiW * 0.90f) && bh > (int)(roiH * 0.90f)) return 0.0f;

  if (density < FIRE_DENSITY_MIN) return 0.0f;
  if (coreR < FIRE_CORE_RATIO_MIN) return 0.0f;
  if (avgY < FIRE_AVGY_MIN) return 0.0f;

  float conf = area * FIRE_W_AREA + density * FIRE_W_DENSITY + coreR * FIRE_W_CORE + strongR * FIRE_W_STRONG;
  if (conf > 1.0f) conf = 1.0f;
  if (conf < 0.0f) conf = 0.0f;

  float strongBoost = 0.35f + 0.65f * fminf(1.0f, strongR / 0.10f);
  conf *= strongBoost;

  if (conf > 1.0f) conf = 1.0f;
  if (conf < 0.0f) conf = 0.0f;
  return conf;
}

static float detectFireOnce() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return 0.0f;

  float conf = scoreFireFromFb(fb);

  esp_camera_fb_return(fb);

  if (conf < 0.0f) conf = 0.0f;
  if (conf > 1.0f) conf = 1.0f;
  return conf;
}

static float detectFireWithEmissiveProbe(bool allowProbe) {
  dbg_probeDelta = 0.0f;

  float c1 = detectFireOnce();
  float c = c1;

  if (allowProbe && c1 > FIRE_PROBE_C1_MIN && c1 < FIRE_PROBE_C1_MAX) {
    setFlash(LED_DETECT_LEVEL);
    delay(FIRE_PROBE_DELAY_MS);
    float c2 = detectFireOnce();
    setFlash(0);

    float d = c2 - c1;
    dbg_probeDelta = d;

    float mix = 0.5f * (c1 + c2);

    float penalty = 1.0f;
    if (d > FIRE_PROBE_D_START) {
      float t = (d - FIRE_PROBE_D_START) / FIRE_PROBE_D_RANGE;
      if (t < 0.0f) t = 0.0f;
      if (t > 1.0f) t = 1.0f;
      penalty = 1.0f - (FIRE_PROBE_PENALTY_STRENGTH * t);
    }

    c = mix * penalty;

    if (d > FIRE_PROBE_D_HARD) c *= FIRE_PROBE_HARD_MULT;
  }

  if (c < 0.0f) c = 0.0f;
  if (c > 1.0f) c = 1.0f;
  return c;
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
  String statusLine = client.readStringUntil('\n');
  if (statusLine.startsWith("HTTP/1.1 ")) code = statusLine.substring(9, 12).toInt();
  else if (statusLine.startsWith("HTTP/1.0 ")) code = statusLine.substring(9, 12).toInt();

  client.stop();

  if (needsFree) free(jpgBuf);
  esp_camera_fb_return(fb);

  return (code >= 200 && code < 300);
}

static int argInt(const char *k, int defv) {
  if (!server.hasArg(k)) return defv;
  return server.arg(k).toInt();
}

static void handleCam() {
  if (!cam) {
    server.send(500, "text/plain", "no sensor");
    return;
  }

  int ae = argInt("ae", 0);
  int ag = argInt("ag", 0);
  int wb = argInt("wb", 0);
  int awb = argInt("awb", 0);
  int aec = argInt("aec", 500);
  int agc = argInt("agc", 0);

  cam->set_exposure_ctrl(cam, ae ? 1 : 0);
  cam->set_gain_ctrl(cam, ag ? 1 : 0);
  cam->set_whitebal(cam, wb ? 1 : 0);
  cam->set_awb_gain(cam, awb ? 1 : 0);
  if (!ae) cam->set_aec_value(cam, aec);
  if (!ag) cam->set_agc_gain(cam, agc);

  char out[220];
  snprintf(out, sizeof(out), "OK ae=%d ag=%d wb=%d awb=%d aec=%d agc=%d", ae, ag, wb, awb, aec, agc);
  server.send(200, "text/plain", out);
}

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

  esp_err_t err = esp_camera_init(&c);

  sensor_t *s = esp_camera_sensor_get();
  cam = s;

  if (err == ESP_OK && s) {
    s->set_exposure_ctrl(s, 0);
    s->set_gain_ctrl(s, 0);
    s->set_whitebal(s, 0);
    s->set_awb_gain(s, 0);
    s->set_aec_value(s, 500);
    s->set_agc_gain(s, 0);
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 1);
  }

  for (int i = 0; i < 3; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(30);
  }
}

static void handleRoot() {
  IPAddress ip = (WiFi.getMode() == WIFI_AP) ? WiFi.softAPIP() : WiFi.localIP();

  char page[1900];
  snprintf(page, sizeof(page),
           "<!doctype html><html><head><meta charset='utf-8'>"
           "<meta name='viewport' content='width=device-width,initial-scale=1'>"
           "<title>ESP32 LIVE</title>"
           "<style>"
           "body{margin:0;font-family:ui-monospace,Consolas,monospace;background:#0b0f14;color:#e6edf3;padding:16px;}"
           ".card{background:#121925;border:1px solid #1f2a3a;border-radius:10px;padding:12px;max-width:1000px;}"
           "pre{white-space:pre-wrap;word-break:break-word;margin:0;}"
           ".muted{color:#9fb1c1;font-size:12px;margin:0 0 10px 0;}"
           "a{color:#9fb1c1;text-decoration:none;}"
           "</style></head><body>"
           "<div class='card'>"
           "<div class='muted'>IP: %u.%u.%u.%u · <a href='/status'>/status</a> · <a href='/cam?ae=0&ag=0&wb=0&awb=0&aec=500&agc=0'>cam lock</a> · <a href='/cam?ae=1&ag=1&wb=1&awb=1'>cam auto</a></div>"
           "<pre id='out'>loading...</pre>"
           "</div>"
           "<script>"
           "async function tick(){"
           " try{const r=await fetch('/status',{cache:'no-store'});"
           " const t=await r.text();"
           " document.getElementById('out').textContent=t;"
           " }catch(e){document.getElementById('out').textContent='(disconnected)';}"
           "}"
           "setInterval(tick,600);tick();"
           "</script></body></html>",
           ip[0], ip[1], ip[2], ip[3]);

  server.send(200, "text/html", page);
}

static void handleStatus() {
  char out[1100];

  uint32_t now = millis();
  uint32_t rxAgo = (lastRxMs == 0) ? 0 : (now - lastRxMs) / 1000;
  uint32_t fireAgo = (lastFireMs == 0) ? 0 : (now - lastFireMs) / 1000;

  snprintf(out, sizeof(out),
           "Last line: %s\n\n"
           "v=%d  loco=%d  gas=%d\n"
           "mq2=%d  mq7=%d  mq135=%d\n"
           "gasRaw=%d  rise=%d  pir=%d\n"
           "dist=%d  ldr=%d\n\n"
           "Last RX: %lus ago\n"
           "Fire: %.1f%% (%lus ago)\n"
           "FireState: %d\n"
           "confRaw=%.3f  confEma=%.3f\n"
           "probeDelta=%.3f\n"
           "firePix=%d  bw=%d  bh=%d\n"
           "density=%.3f  core=%.3f  avgY=%.1f\n"
           "Flash: %s\n"
           "Heap: %u\n",
           lastLine[0] ? lastLine : "(none)",
           live.valid ? live.version : 0,
           live.valid ? live.locoState : 0,
           live.valid ? live.gasState : 0,
           live.valid ? live.mq2 : 0,
           live.valid ? live.mq7 : 0,
           live.valid ? live.mq135 : 0,
           live.valid ? live.gasRaw : 0,
           live.valid ? live.gasRise : 0,
           live.valid ? live.pir : 0,
           live.valid ? live.dist : -1,
           live.valid ? live.ldr : 0,
           (unsigned long)rxAgo,
           lastFirePercent,
           (unsigned long)fireAgo,
           fireStateHys ? 1 : 0,
           dbg_confRaw, dbg_confEma,
           dbg_probeDelta,
           dbg_firePix, dbg_bw, dbg_bh,
           dbg_density, dbg_core, dbg_avgY,
           flashOn ? "ON" : "OFF",
           (unsigned)ESP.getFreeHeap());

  server.send(200, "text/plain", out);
}

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
  server.on("/cam", handleCam);
  server.begin();

  rxBuf[0] = 0;
  lastLine[0] = 0;

  Serial.println("ESP32 READY");
}

void loop() {
  server.handleClient();

  bool gotLine = false;

  while (Link.available()) {
    char c = (char)Link.read();

    if (c == '\n') {
      rxBuf[rxLen] = 0;

      strncpy(lastLine, rxBuf, MAX_LINE - 1);
      lastLine[MAX_LINE - 1] = 0;
      lastRxMs = millis();

      parseLiveLine(rxBuf);

      rxLen = 0;
      gotLine = true;
    } else if (c != '\r') {
      if (rxLen < MAX_LINE - 1) rxBuf[rxLen++] = c;
      else rxLen = 0;
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
    if (!flashOn && live.ldr > LDR_FLASH_ON) flashOn = true;
    if (flashOn && live.ldr < LDR_FLASH_OFF) flashOn = false;
    setFlash(flashOn ? LED_ON_LEVEL : 0);
  } else {
    setFlash(0);
    flashOn = false;
  }

  static uint32_t lastFireCheck = 0;
  uint32_t now = millis();
  if (now - lastFireCheck >= FIRE_CHECK_INTERVAL_MS) {
    lastFireCheck = now;

    bool allowProbe = (dbg_confRaw > 0.20f);

    uint8_t saved = flashOn ? LED_ON_LEVEL : 0;
    if (saved) setFlash(0);

    float raw = detectFireWithEmissiveProbe(allowProbe);

    if (saved) setFlash(saved);

    dbg_confRaw = raw;

    float a = FIRE_EMA_ALPHA;
    if (a < 0.0f) a = 0.0f;
    if (a > 1.0f) a = 1.0f;

    fireEma = fireEma * (1.0f - a) + raw * a;
    if (fireEma < 0.0f) fireEma = 0.0f;
    if (fireEma > 1.0f) fireEma = 1.0f;
    dbg_confEma = fireEma;

    int assist = (live.valid ? live.gasRise : 0);

    float onTh = (assist >= GAS_RISE_ASSIST_TH) ? FIRE_ON_TH_ASSIST : FIRE_ON_TH_STRONG;
    float offTh = (assist >= GAS_RISE_ASSIST_TH) ? FIRE_OFF_TH_ASSIST : FIRE_OFF_TH_STRONG;

    static uint8_t onHits = 0;
    static uint8_t offHits = 0;

    if (raw >= FIRE_RAW_INSTANT_TH) {
      fireStateHys = true;
      onHits = 10;
      offHits = 0;
    }

    if (fireEma >= onTh) {
      if (onHits < 10) onHits++;
    } else {
      onHits = 0;
    }

    if (fireEma <= offTh) {
      if (offHits < 10) offHits++;
    } else {
      offHits = 0;
    }

    if (!fireStateHys && onHits >= FIRE_ON_HITS) fireStateHys = true;
    if (fireStateHys && offHits >= FIRE_OFF_HITS) fireStateHys = false;

    lastFirePercent = fireEma * 100.0f;
    lastFireMs = now;

    float conf01 = lastFirePercent / 100.0f;

    Link.print("FIRE32,confidence=");
    Link.println(conf01, 3);

    Serial.print("UNO_TX conf=");
    Serial.print(conf01, 3);
    Serial.print(" raw=");
    Serial.print(dbg_confRaw, 3);
    Serial.print(" ema=");
    Serial.print(dbg_confEma, 3);
    Serial.print(" d=");
    Serial.print(dbg_probeDelta, 3);
    Serial.print(" firePix=");
    Serial.print(dbg_firePix);
    Serial.print(" bw=");
    Serial.print(dbg_bw);
    Serial.print(" bh=");
    Serial.print(dbg_bh);
    Serial.print(" dens=");
    Serial.print(dbg_density, 3);
    Serial.print(" core=");
    Serial.print(dbg_core, 3);
    Serial.print(" y=");
    Serial.print(dbg_avgY, 1);
    Serial.print(" gasRise=");
    Serial.print(assist);
    Serial.print(" state=");
    Serial.println(fireStateHys ? 1 : 0);

    if (fireStateHys && !sentForThisFire) {
      bool ok = uploadFireFrame(fireEma);
      sentForThisFire = ok ? true : false;
    }
    if (!fireStateHys) sentForThisFire = false;
  }

  yield();
}