// Robotic arm controller — ESP32 + PCA9685 + Pixy2
// 純 HTTP server 架構：/vision 回傳 Pixy 座標、/execute 收角度後執行
// 加值：S-curve 平滑運動、視覺 PID 對準、Pixy 多幀中位數、PCA9685 批次寫

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>

// ── 設定 ─────────────────────────────────────────
namespace cfg {
  // WiFi
  constexpr char SSID[]          = "1234567";
  constexpr char PASS[]          = "asdfghjk";
  constexpr char MDNS_HOSTNAME[] = "banyus";   // → banyus.local
  constexpr uint16_t HTTP_PORT   = 80;

  // I2C
  constexpr uint8_t  PWM_ADDR  = 0x40;
  constexpr uint8_t  PIXY_ADDR = 0x54;
  constexpr uint8_t  SDA_PWM   = 21, SCL_PWM  = 22;
  constexpr uint8_t  SDA_PIXY  = 4,  SCL_PIXY = 5;

  // Servo
  constexpr uint16_t PWM_FREQ   = 50;
  constexpr uint16_t PWM_MIN    = 150;   // 0°
  constexpr uint16_t PWM_MAX    = 600;   // 180°
  constexpr uint8_t  SERVO_LO   = 5;
  constexpr uint8_t  SERVO_HI   = 175;
  constexpr uint8_t  GRIPPER_HI = 90;

  // Home
  constexpr uint8_t HOME_CH0 = 90, HOME_CH1 = 90, HOME_CH2 = 120, HOME_CH3 = 5;

  // Motion
  constexpr uint16_t MOVE_STEPS      = 40;
  constexpr uint16_t MOVE_MS         = 1000;   // 階段一般動作時間
  constexpr uint16_t MOVE_FAST_MS    = 350;    // PID 小幅修正時間
  constexpr uint16_t MOVE_INIT_MS    = 2000;   // 開機歸位
  constexpr uint16_t PHASE_SETTLE_MS = 150;    // 階段間 settle

  // Vision (median sampling 過濾單幀雜訊)
  constexpr uint8_t  PIXY_SIG_TARGET    = 1;
  constexpr uint8_t  PIXY_SIG_DEST      = 2;
  constexpr uint8_t  PIXY_MAX_BLOCKS    = 4;
  constexpr uint8_t  PIXY_READ_DELAY_MS = 5;
  constexpr uint8_t  MEDIAN_SAMPLES     = 3;
  constexpr uint8_t  MEDIAN_GAP_MS      = 15;

  // PID（視覺對準，僅作用於 ch0）
  constexpr float    KP            = 0.43f;
  constexpr float    KI            = 0.05f;
  constexpr float    KD            = 0.10f;
  constexpr float    I_MAX         = 30.0f;
  constexpr uint8_t  PID_MAX_ITER  = 3;
  constexpr uint8_t  PID_TOLERANCE = 5;        // 像素
  constexpr uint16_t PID_SETTLE_MS = 220;

  // 系統
  constexpr uint32_t WDT_TIMEOUT_MS = 30000;
  constexpr uint8_t  PCA_LED0_ON_L  = 0x06;
}

// ── 全域 ─────────────────────────────────────────
struct Block {
  int signature = 0;
  int x = -1, y = -1, width = 0, height = 0;
  bool valid() const { return x >= 0; }
};

static WebServer              server(cfg::HTTP_PORT);
static Adafruit_PWMServoDriver pwm(cfg::PWM_ADDR);
static int                    cur[4] = {cfg::HOME_CH0, cfg::HOME_CH1, cfg::HOME_CH2, cfg::HOME_CH3};
static volatile bool          busy   = false;     // /execute 執行中時擋掉 /vision 與重入

// ── 工具 ─────────────────────────────────────────
static inline int  angleToPWM(int a) { return map(a, 0, 180, cfg::PWM_MIN, cfg::PWM_MAX); }
static inline void feedWDT()         { esp_task_wdt_reset(); }

// 5 階 smoothstep（S-curve）：起終點 v 與 a 皆為 0，jerk-limited
static inline float sCurveRatio(float t) {
  return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

// 一次 I2C 寫入 4 軸 PWM（PCA9685 auto-increment）
static void writePWM4(const int angles[4]) {
  Wire.beginTransmission(cfg::PWM_ADDR);
  Wire.write(cfg::PCA_LED0_ON_L);
  for (int ch = 0; ch < 4; ch++) {
    const int a = constrain(angles[ch], 0, cfg::SERVO_HI);
    const uint16_t v = angleToPWM(a);
    Wire.write(0);                 // ON_L
    Wire.write(0);                 // ON_H
    Wire.write(v & 0xFF);          // OFF_L
    Wire.write((v >> 8) & 0xFF);   // OFF_H
  }
  Wire.endTransmission();
}

// ── S-curve 平滑運動（block but 期間仍服務 HTTP） ────
static void moveSmooth(const int target[4], int total_ms = cfg::MOVE_MS) {
  const int steps = cfg::MOVE_STEPS;
  const int dt    = max(1, total_ms / steps);
  int start[4]; memcpy(start, cur, sizeof(cur));

  for (int s = 1; s <= steps; s++) {
    const float r = sCurveRatio((float)s / steps);
    int a[4];
    for (int ch = 0; ch < 4; ch++) {
      a[ch] = start[ch] + (int)((target[ch] - start[ch]) * r);
    }
    writePWM4(a);
    feedWDT();
    server.handleClient();   // 維持並發
    delay(dt);
  }
  memcpy(cur, target, sizeof(cur));
}

// ── Pixy2 I2C 讀取 ───────────────────────────────
static Block getBlockBySig(int sig) {
  Block b;
  Wire1.beginTransmission(cfg::PIXY_ADDR);
  Wire1.write(0x20); Wire1.write(0x00); Wire1.write(cfg::PIXY_MAX_BLOCKS);
  Wire1.endTransmission();
  delay(cfg::PIXY_READ_DELAY_MS);

  int count = 0;
  Wire1.requestFrom(cfg::PIXY_ADDR, (uint8_t)2);
  if (Wire1.available() >= 2) { Wire1.read(); count = Wire1.read(); }
  if (count <= 0) return b;

  for (int i = 0; i < count; i++) {
    if (Wire1.requestFrom(cfg::PIXY_ADDR, (uint8_t)14) < 14) continue;
    if (Wire1.available() < 14) continue;
    for (int k = 0; k < 4; k++) Wire1.read();
    const int s = Wire1.read() | (Wire1.read() << 8);
    const int x = Wire1.read() | (Wire1.read() << 8);
    const int y = Wire1.read() | (Wire1.read() << 8);
    const int w = Wire1.read() | (Wire1.read() << 8);
    const int h = Wire1.read() | (Wire1.read() << 8);
    if (s == sig) { b = {s, x, y, w, h}; return b; }
  }
  return b;
}

static int medianOf(int* a, int n) {
  for (int i = 0; i < n - 1; i++) {
    int mi = i;
    for (int j = i + 1; j < n; j++) if (a[j] < a[mi]) mi = j;
    if (mi != i) { int t = a[i]; a[i] = a[mi]; a[mi] = t; }
  }
  return a[n / 2];
}

static Block getBlockMedian(int sig) {
  int xs[8], ys[8], ws[8], hs[8];
  int n = 0;
  const int N = min((int)cfg::MEDIAN_SAMPLES, 8);
  for (int i = 0; i < N; i++) {
    feedWDT();
    Block b = getBlockBySig(sig);
    if (b.valid()) {
      xs[n] = b.x; ys[n] = b.y; ws[n] = b.width; hs[n] = b.height;
      n++;
    }
    delay(cfg::MEDIAN_GAP_MS);
  }
  if (n == 0) return Block{};
  Block r;
  r.signature = sig;
  r.x      = medianOf(xs, n);
  r.y      = medianOf(ys, n);
  r.width  = medianOf(ws, n);
  r.height = medianOf(hs, n);
  return r;
}

// ── 視覺 PID 對準（pick 後做 ch0 修正） ──────────
static void visualPID(int target_u) {
  float pid_i = 0.0f;
  int   prev_err = 0;

  for (uint8_t iter = 0; iter < cfg::PID_MAX_ITER; iter++) {
    feedWDT();
    server.handleClient();
    delay(cfg::PID_SETTLE_MS);

    const Block b = getBlockMedian(cfg::PIXY_SIG_TARGET);
    if (!b.valid()) break;

    const int err = target_u - b.x;
    if (abs(err) < cfg::PID_TOLERANCE) break;

    pid_i = constrain(pid_i + (float)err, -cfg::I_MAX, cfg::I_MAX);
    const int delta = (int)lroundf(
      cfg::KP * (float)err +
      cfg::KI * pid_i +
      cfg::KD * (float)(err - prev_err));
    prev_err = err;
    if (delta == 0) break;

    const int ch0 = constrain(cur[0] + delta, cfg::SERVO_LO, cfg::SERVO_HI);
    Serial.printf("  PID iter=%u err=%d i=%.1f Δ=%d ch0=%d\n",
                  iter, err, pid_i, delta, ch0);
    const int target[4] = {ch0, cur[1], cur[2], cur[3]};
    moveSmooth(target, cfg::MOVE_FAST_MS);
  }
}

// ── 執行 5 階段（可選 target_u 啟動 pick 後的 PID） ─
static void executeAngles(JsonDocument& doc) {
  static const char* const PHASES[] = {"pick", "grip", "place", "release", "home"};
  const int target_u = doc["target_u"] | -1;

  for (int i = 0; i < 5; i++) {
    if (!doc[PHASES[i]].is<JsonObject>()) continue;
    JsonObject p = doc[PHASES[i]];
    const int a[4] = {
      constrain((int)p["ch0"], cfg::SERVO_LO, cfg::SERVO_HI),
      constrain((int)p["ch1"], cfg::SERVO_LO, cfg::SERVO_HI),
      constrain((int)p["ch2"], cfg::SERVO_LO, cfg::SERVO_HI),
      constrain((int)p["ch3"], cfg::SERVO_LO, cfg::GRIPPER_HI),
    };
    Serial.printf("▶ %s → ch0=%d ch1=%d ch2=%d ch3=%d\n",
                  PHASES[i], a[0], a[1], a[2], a[3]);
    moveSmooth(a, cfg::MOVE_MS);

    if (target_u > 0 && strcmp(PHASES[i], "pick") == 0) {
      Serial.println("  🎯 視覺 PID 修正");
      visualPID(target_u);
    }

    // 階段間 settle，期間繼續服務 HTTP
    const unsigned long t0 = millis();
    while (millis() - t0 < cfg::PHASE_SETTLE_MS) {
      feedWDT();
      server.handleClient();
      delay(5);
    }
  }
}

// ── HTTP handlers ────────────────────────────────
static void sendJson(int code, const char* body) {
  server.send(code, "application/json", body);
}

// POST /execute  — 收 5 階段角度 JSON（可附 "target_u" 開啟 PID）
static void handleExecute() {
  if (busy) { sendJson(503, "{\"error\":\"busy\"}"); return; }
  if (!server.hasArg("plain")) { sendJson(400, "{\"error\":\"no body\"}"); return; }

  const String body = server.arg("plain");
  Serial.printf("📥 /execute: %s\n", body.c_str());

  DynamicJsonDocument doc(2048);
  const DeserializationError err = deserializeJson(doc, body);
  if (err) {
    Serial.printf("⚠ JSON 解析失敗: %s\n", err.c_str());
    sendJson(400, "{\"error\":\"bad json\"}");
    return;
  }

  busy = true;
  sendJson(200, "{\"ok\":true}");     // 立刻回，動作於本 handler 內阻塞執行
  executeAngles(doc);
  busy = false;
  Serial.println("✅ 動作完成");
}

// GET /vision  — 回傳 Pixy 中位數座標
static void handleVision() {
  if (busy) { sendJson(503, "{\"error\":\"busy\"}"); return; }

  const Block target = getBlockMedian(cfg::PIXY_SIG_TARGET);
  const Block dest   = getBlockMedian(cfg::PIXY_SIG_DEST);

  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"target_found\":%s,\"dest_found\":%s,"
           "\"tx\":%d,\"ty\":%d,\"dx\":%d,\"dy\":%d}",
           target.valid() ? "true" : "false",
           dest.valid()   ? "true" : "false",
           target.x, target.y, dest.x, dest.y);
  server.send(200, "application/json", buf);
}

// GET /health
static void handleHealth() {
  char buf[80];
  snprintf(buf, sizeof(buf),
           "{\"busy\":%s,\"ch\":[%d,%d,%d,%d]}",
           busy ? "true" : "false",
           cur[0], cur[1], cur[2], cur[3]);
  sendJson(200, buf);
}

static void handleNotFound() {
  sendJson(404, "{\"error\":\"not found\"}");
}

// ── setup / loop ─────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🚀 啟動中...");

  esp_task_wdt_deinit();
  esp_task_wdt_config_t wdt_cfg = {
    .timeout_ms     = cfg::WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic  = true,
  };
  esp_task_wdt_init(&wdt_cfg);
  esp_task_wdt_add(nullptr);

  Wire.begin(cfg::SDA_PWM, cfg::SCL_PWM);
  pwm.begin();
  pwm.setPWMFreq(cfg::PWM_FREQ);

  // 開機 home（S-curve 平滑）
  const int home[4] = {cfg::HOME_CH0, cfg::HOME_CH1, cfg::HOME_CH2, cfg::HOME_CH3};
  moveSmooth(home, cfg::MOVE_INIT_MS);
  Serial.println("✅ 伺服馬達初始化完成");

  Wire1.begin(cfg::SDA_PIXY, cfg::SCL_PIXY);
  Serial.println("✅ Pixy2 I2C 初始化完成");

  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg::SSID, cfg::PASS);
  Serial.print("WiFi 連線中");
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    feedWDT(); delay(500); Serial.print('.');
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n❌ WiFi 失敗，重啟");
    ESP.restart();
  }
  Serial.printf("\n✅ WiFi 連線成功 %s\n", WiFi.localIP().toString().c_str());

  if (MDNS.begin(cfg::MDNS_HOSTNAME)) {
    Serial.printf("✅ mDNS: http://%s.local\n", cfg::MDNS_HOSTNAME);
  } else {
    Serial.println("⚠ mDNS 啟動失敗");
  }

  server.on("/execute", HTTP_POST, handleExecute);
  server.on("/vision",  HTTP_GET,  handleVision);
  server.on("/health",  HTTP_GET,  handleHealth);
  server.onNotFound(handleNotFound);
  server.begin();
  MDNS.addService("http", "tcp", cfg::HTTP_PORT);
  Serial.printf("✅ HTTP server on :%u (/execute, /vision, /health)\n",
                cfg::HTTP_PORT);
}

void loop() {
  feedWDT();
  if (WiFi.status() != WL_CONNECTED) { WiFi.reconnect(); delay(500); return; }
  server.handleClient();
}
