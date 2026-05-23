// Robotic arm controller — ESP32 + PCA9685 + Pixy2 + HTTP backend
// 非阻塞 FSM + S-curve + 多幀中位數 + 完整 PID + PCA9685 批次寫入 + 串流 JSON

#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>

// ── 設定常數 ──────────────────────────────────────
namespace cfg {
  // WiFi / Server
  constexpr char SSID[]       = "1234567";
  constexpr char PASS[]       = "asdfghjk";
  constexpr char SERVER_BASE[]   = "http://10.20.171.77:5000";  // Flask base URL
  constexpr char REPORT_PATH[]   = "/report";                    // ESP → Flask 回報座標
  constexpr char MDNS_HOSTNAME[] = "banyus";        // → banyus.local
  constexpr uint16_t HTTP_TIMEOUT_MS = 5000;
  constexpr uint8_t  HTTP_MAX_RETRY  = 2;
  constexpr uint16_t HTTP_LISTEN_PORT = 80;          // ESP32 端 /execute /health

  // I2C
  constexpr uint8_t  PWM_ADDR   = 0x40;
  constexpr uint8_t  PIXY_ADDR  = 0x54;
  constexpr uint8_t  SDA_PWM    = 21;
  constexpr uint8_t  SCL_PWM    = 22;
  constexpr uint8_t  SDA_PIXY   = 4;
  constexpr uint8_t  SCL_PIXY   = 5;

  // Servo
  constexpr uint16_t PWM_FREQ   = 50;
  constexpr uint16_t PWM_MIN    = 150;   // 0°
  constexpr uint16_t PWM_MAX    = 600;   // 180°
  constexpr uint8_t  SERVO_LO   = 5;
  constexpr uint8_t  SERVO_HI   = 175;
  constexpr uint8_t  GRIPPER_HI = 90;

  // Home
  constexpr uint8_t  HOME_CH0   = 90;
  constexpr uint8_t  HOME_CH1   = 90;
  constexpr uint8_t  HOME_CH2   = 120;
  constexpr uint8_t  HOME_CH3   = 5;

  // Motion
  constexpr uint16_t MOVE_STEPS      = 40;
  constexpr uint16_t MOVE_MS         = 1200;
  constexpr uint16_t MOVE_FAST_MS    = 400;
  constexpr uint16_t MOVE_INIT_MS    = 2000;
  constexpr uint16_t PHASE_SETTLE_MS = 300;

  // Vision
  constexpr uint16_t VISION_INTERVAL_MS = 3000;
  constexpr uint8_t  PIXY_SIG_TARGET    = 1;
  constexpr uint8_t  PIXY_SIG_DEST      = 2;
  constexpr uint8_t  PIXY_MAX_BLOCKS    = 4;
  constexpr uint8_t  PIXY_READ_DELAY_MS = 5;
  constexpr uint8_t  MEDIAN_SAMPLES     = 3;
  constexpr uint8_t  MEDIAN_GAP_MS      = 20;

  // PID
  constexpr float    KP            = 0.43f;
  constexpr float    KI            = 0.05f;
  constexpr float    KD            = 0.10f;
  constexpr float    I_MAX         = 30.0f;
  constexpr uint8_t  PID_MAX_ITER  = 3;
  constexpr uint8_t  PID_TOLERANCE = 5;     // 像素
  constexpr uint16_t PID_SETTLE_MS = 300;

  // Watchdog / JSON
  constexpr uint32_t WDT_TIMEOUT_MS = 30000;
  constexpr size_t   JSON_BUF_SIZE  = 2048;

  // PCA9685 register
  constexpr uint8_t  PCA_LED0_ON_L  = 0x06;
}

// ── 型別 ──────────────────────────────────────────
struct Block {
  int signature = 0;
  int x = -1, y = -1, width = 0, height = 0;
  bool valid() const { return x >= 0; }
};

struct Motion {
  bool active = false;
  int  start_a[4]{};
  int  target_a[4]{};
  int  total_ms = 0;
  unsigned long t0 = 0;
  int  last_step = -1;
};

struct PhaseAngles {
  int  ch[4];
  bool present = false;
};

enum class Stage : uint8_t { IDLE, EXECUTE };
enum class Sub   : uint8_t { PHASE_MOVE, PHASE_SETTLE, PID_SETTLE, PID_MOVE };

struct SysState {
  Stage stage = Stage::IDLE;
  Sub   sub   = Sub::PHASE_MOVE;
  int   phase_idx = 0;
  int   target_u  = -1;
  unsigned long stage_t    = 0;
  unsigned long lastVision = 0;

  // PID
  uint8_t pid_iter     = 0;
  float   pid_i        = 0.0f;
  int     pid_prev_err = 0;
};

// ── 全域 ──────────────────────────────────────────
static Adafruit_PWMServoDriver pwm(cfg::PWM_ADDR);
static WebServer    server(cfg::HTTP_LISTEN_PORT);
static int          cur[4] = {cfg::HOME_CH0, cfg::HOME_CH1, cfg::HOME_CH2, cfg::HOME_CH3};
static Motion       motion;
static const char* const PHASE_NAMES[] = {"pick", "grip", "place", "release", "home"};
static constexpr uint8_t NUM_PHASES    = 5;
static PhaseAngles  phases[NUM_PHASES];
static SysState     S;

// ── 工具 ──────────────────────────────────────────
static inline int  angleToPWM(int a) { return map(a, 0, 180, cfg::PWM_MIN, cfg::PWM_MAX); }
static inline void feedWDT()         { esp_task_wdt_reset(); }

// 5 階 smoothstep（S 曲線）：起終點 v 與 a 皆為 0，jerk-limited
static inline float sCurveRatio(float t) {
  return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

// 一次 I2C 寫完 4 軸 PWM（PCA9685 auto-increment）
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

// ── 動作 FSM ──────────────────────────────────────
static void startMotion(int ch0, int ch1, int ch2, int ch3,
                        int total_ms = cfg::MOVE_MS) {
  motion.target_a[0] = ch0;
  motion.target_a[1] = ch1;
  motion.target_a[2] = ch2;
  motion.target_a[3] = ch3;
  memcpy(motion.start_a, cur, sizeof(cur));
  motion.total_ms  = max(50, total_ms);
  motion.t0        = millis();
  motion.last_step = -1;
  motion.active    = true;
}

static void tickMotion() {
  if (!motion.active) return;
  const unsigned long elapsed = millis() - motion.t0;

  if (elapsed >= (unsigned long)motion.total_ms) {
    memcpy(cur, motion.target_a, sizeof(cur));
    writePWM4(cur);
    motion.active = false;
    return;
  }

  const int step = (int)((elapsed * cfg::MOVE_STEPS) / motion.total_ms);
  if (step == motion.last_step) return;
  motion.last_step = step;

  const float r = sCurveRatio((float)step / cfg::MOVE_STEPS);
  int a[4];
  for (int ch = 0; ch < 4; ch++) {
    a[ch] = motion.start_a[ch] + (int)((motion.target_a[ch] - motion.start_a[ch]) * r);
  }
  writePWM4(a);
}

// ── Pixy2 I2C 讀取 ────────────────────────────────
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

// 多幀中位數，過濾單幀雜訊
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

// ── JSON 解析 → phases[] ──────────────────────────
static bool parsePhases(JsonDocument& doc) {
  for (uint8_t i = 0; i < NUM_PHASES; i++) {
    JsonVariant v = doc[PHASE_NAMES[i]];
    if (!v.is<JsonObject>()) { phases[i].present = false; continue; }
    phases[i].ch[0]   = constrain((int)v["ch0"], cfg::SERVO_LO, cfg::SERVO_HI);
    phases[i].ch[1]   = constrain((int)v["ch1"], cfg::SERVO_LO, cfg::SERVO_HI);
    phases[i].ch[2]   = constrain((int)v["ch2"], cfg::SERVO_LO, cfg::SERVO_HI);
    phases[i].ch[3]   = constrain((int)v["ch3"], cfg::SERVO_LO, cfg::GRIPPER_HI);
    phases[i].present = true;
  }
  return true;
}

// ── HTTP：純回報座標到 Flask /report（fire-and-forget） ─────
// 角度由 Flask 在 AUTO 模式下主動 POST 回 /sequence
static bool reportCoords(int tx, int ty, int dx, int dy) {
  char body[96];
  const int n = snprintf(body, sizeof(body),
                         "{\"tx\":%d,\"ty\":%d,\"dx\":%d,\"dy\":%d}",
                         tx, ty, dx, dy);
  if (n <= 0 || n >= (int)sizeof(body)) return false;

  const String url = String(cfg::SERVER_BASE) + cfg::REPORT_PATH;
  Serial.printf("📤 %s %s\n", url.c_str(), body);

  HTTPClient http;
  http.setTimeout(cfg::HTTP_TIMEOUT_MS);
  http.setReuse(false);

  uint16_t backoff = 300;
  for (uint8_t attempt = 0; attempt <= cfg::HTTP_MAX_RETRY; attempt++) {
    if (!http.begin(url)) { delay(200); continue; }
    http.addHeader("Content-Type", "application/json");
    const int code = http.POST((uint8_t*)body, n);
    http.end();
    if (code == 200) return true;
    Serial.printf("⚠ /report code=%d (重試 %u/%u, %ums)\n",
                  code, attempt + 1, cfg::HTTP_MAX_RETRY, backoff);
    feedWDT();
    delay(backoff);
    backoff = min<uint16_t>(backoff * 2, 2000);
  }
  return false;
}

// ── 階段切換 ──────────────────────────────────────
static void enterPhase(int idx) {
  while (idx < NUM_PHASES && !phases[idx].present) idx++;
  if (idx >= NUM_PHASES) {
    Serial.println("✅ 動作完成");
    S.stage = Stage::IDLE;
    S.lastVision = millis();
    return;
  }
  const auto& p = phases[idx];
  Serial.printf("▶ %s → ch0=%d ch1=%d ch2=%d ch3=%d\n",
                PHASE_NAMES[idx], p.ch[0], p.ch[1], p.ch[2], p.ch[3]);
  S.phase_idx = idx;
  S.sub = Sub::PHASE_MOVE;
  startMotion(p.ch[0], p.ch[1], p.ch[2], p.ch[3], cfg::MOVE_MS);
}

// ── 執行 FSM ──────────────────────────────────────
static void tickExecute() {
  if (S.stage != Stage::EXECUTE) return;

  switch (S.sub) {
    case Sub::PHASE_MOVE:
      if (motion.active) return;
      S.stage_t = millis();
      S.sub     = Sub::PHASE_SETTLE;
      break;

    case Sub::PHASE_SETTLE:
      if (millis() - S.stage_t < cfg::PHASE_SETTLE_MS) return;
      if (S.target_u > 0 && strcmp(PHASE_NAMES[S.phase_idx], "pick") == 0) {
        Serial.println("  🎯 視覺修正中...");
        S.pid_iter     = 0;
        S.pid_i        = 0.0f;
        S.pid_prev_err = 0;
        S.stage_t      = millis();
        S.sub          = Sub::PID_SETTLE;
      } else {
        enterPhase(S.phase_idx + 1);
      }
      break;

    case Sub::PID_SETTLE: {
      if (millis() - S.stage_t < cfg::PID_SETTLE_MS) return;
      if (S.pid_iter >= cfg::PID_MAX_ITER) {
        enterPhase(S.phase_idx + 1);
        return;
      }
      const Block b = getBlockMedian(cfg::PIXY_SIG_TARGET);
      if (!b.valid()) { enterPhase(S.phase_idx + 1); return; }
      const int err = S.target_u - b.x;
      if (abs(err) < cfg::PID_TOLERANCE) {
        enterPhase(S.phase_idx + 1);
        return;
      }
      S.pid_i = constrain(S.pid_i + (float)err, -cfg::I_MAX, cfg::I_MAX);
      const int delta = (int)lroundf(
        cfg::KP * (float)err +
        cfg::KI * S.pid_i +
        cfg::KD * (float)(err - S.pid_prev_err));
      S.pid_prev_err = err;
      if (delta == 0) { enterPhase(S.phase_idx + 1); return; }
      const int ch0 = constrain(cur[0] + delta, cfg::SERVO_LO, cfg::SERVO_HI);
      Serial.printf("  PID iter=%u err=%d i=%.1f Δ=%d ch0=%d\n",
                    S.pid_iter, err, S.pid_i, delta, ch0);
      S.pid_iter++;
      startMotion(ch0, cur[1], cur[2], cur[3], cfg::MOVE_FAST_MS);
      S.sub = Sub::PID_MOVE;
      break;
    }

    case Sub::PID_MOVE:
      if (motion.active) return;
      S.stage_t = millis();
      S.sub     = Sub::PID_SETTLE;
      break;
  }
}

// ── IDLE：間隔到了就掃描+POST+進入 EXECUTE ─────────
static void tickIdle() {
  if (S.stage != Stage::IDLE) return;
  if (motion.active) return;
  if (millis() - S.lastVision < cfg::VISION_INTERVAL_MS) return;
  S.lastVision = millis();

  const Block target = getBlockMedian(cfg::PIXY_SIG_TARGET);
  const Block dest   = getBlockMedian(cfg::PIXY_SIG_DEST);
  if (!target.valid()) { Serial.println("👁 找不到目標物"); return; }
  if (!dest.valid())   { Serial.println("👁 找不到放置目的地"); return; }

  Serial.printf("👁 目標:(%d,%d) 目的地:(%d,%d)\n",
                target.x, target.y, dest.x, dest.y);

  // 純回報；Flask 在 AUTO 模式時會主動 POST /sequence 回來觸發執行
  reportCoords(target.x, target.y, dest.x, dest.y);
}

// ── HTTP server handlers ─────────────────────────
// 接受 {"home":{ch0,ch1,ch2,ch3}} / {"pick":{...}} 等單一階段格式。
// 忙碌時回 503；接受後立刻回 200，動作於 tickMotion 背景執行。
static void sendJson(int code, const char* body) {
  server.send(code, "application/json", body);
}

static void handleHealth() {
  char buf[80];
  snprintf(buf, sizeof(buf),
           "{\"motion\":%s,\"stage\":%d,\"ch\":[%d,%d,%d,%d]}",
           motion.active ? "true" : "false",
           (int)S.stage,
           cur[0], cur[1], cur[2], cur[3]);
  sendJson(200, buf);
}

static void handleExecute() {
  if (S.stage != Stage::IDLE || motion.active) {
    sendJson(503, "{\"error\":\"busy\"}");
    return;
  }
  if (!server.hasArg("plain")) {
    sendJson(400, "{\"error\":\"no body\"}");
    return;
  }

  DynamicJsonDocument doc(512);
  const DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    Serial.printf("⚠ /execute JSON 解析失敗: %s\n", err.c_str());
    sendJson(400, "{\"error\":\"bad json\"}");
    return;
  }

  // 取第一個是 object 的鍵
  JsonObject root = doc.as<JsonObject>();
  for (JsonPair p : root) {
    if (!p.value().is<JsonObject>()) continue;
    JsonObject a = p.value().as<JsonObject>();
    const int ch0 = constrain((int)a["ch0"], cfg::SERVO_LO, cfg::SERVO_HI);
    const int ch1 = constrain((int)a["ch1"], cfg::SERVO_LO, cfg::SERVO_HI);
    const int ch2 = constrain((int)a["ch2"], cfg::SERVO_LO, cfg::SERVO_HI);
    const int ch3 = constrain((int)a["ch3"], cfg::SERVO_LO, cfg::GRIPPER_HI);
    Serial.printf("📥 /execute %s → ch0=%d ch1=%d ch2=%d ch3=%d\n",
                  p.key().c_str(), ch0, ch1, ch2, ch3);
    startMotion(ch0, ch1, ch2, ch3, cfg::MOVE_MS);
    sendJson(200, "{\"ok\":true}");
    return;
  }
  sendJson(400, "{\"error\":\"no phase in body\"}");
}

// 接收完整 5 階段 JSON（同 /arm 回應格式 + target_u）→ 啟動視覺序列 FSM
static void handleSequence() {
  if (S.stage != Stage::IDLE || motion.active) {
    sendJson(503, "{\"error\":\"busy\"}");
    return;
  }
  if (!server.hasArg("plain")) {
    sendJson(400, "{\"error\":\"no body\"}");
    return;
  }

  DynamicJsonDocument doc(cfg::JSON_BUF_SIZE);
  const DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    Serial.printf("⚠ /sequence JSON 解析失敗: %s\n", err.c_str());
    sendJson(400, "{\"error\":\"bad json\"}");
    return;
  }

  if (!parsePhases(doc)) {
    sendJson(400, "{\"error\":\"no phases\"}");
    return;
  }
  S.target_u = doc["target_u"] | -1;     // 沒帶就跳過 PID
  S.stage    = Stage::EXECUTE;
  Serial.printf("📥 /sequence 啟動，target_u=%d\n", S.target_u);
  enterPhase(0);
  sendJson(200, "{\"ok\":true}");
}

static void handleNotFound() {
  sendJson(404, "{\"error\":\"not found\"}");
}

// ── WiFi / Watchdog ──────────────────────────────
static bool wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg::SSID, cfg::PASS);
  Serial.print("WiFi 連線中");
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    feedWDT(); delay(500); Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi 連線失敗");
    return false;
  }
  Serial.printf("✅ WiFi 連線成功 %s\n", WiFi.localIP().toString().c_str());

  if (MDNS.begin(cfg::MDNS_HOSTNAME)) {
    Serial.printf("✅ mDNS：http://%s.local\n", cfg::MDNS_HOSTNAME);
  } else {
    Serial.println("⚠ mDNS 啟動失敗");
  }
  return true;
}

static void initWatchdog() {
  esp_task_wdt_deinit();
  esp_task_wdt_config_t w = {
    .timeout_ms     = cfg::WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic  = true,
  };
  esp_task_wdt_init(&w);
  esp_task_wdt_add(nullptr);
}

// ── setup / loop ──────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🚀 啟動中...");

  initWatchdog();

  Wire.begin(cfg::SDA_PWM, cfg::SCL_PWM);
  pwm.begin();
  pwm.setPWMFreq(cfg::PWM_FREQ);

  // 初始 homing：阻塞等到位
  startMotion(cfg::HOME_CH0, cfg::HOME_CH1, cfg::HOME_CH2, cfg::HOME_CH3,
              cfg::MOVE_INIT_MS);
  while (motion.active) { tickMotion(); feedWDT(); delay(5); }
  Serial.println("✅ 伺服馬達初始化完成");

  Wire1.begin(cfg::SDA_PIXY, cfg::SCL_PIXY);
  Serial.println("✅ Pixy2 I2C 初始化完成");

  if (!wifiConnect()) ESP.restart();

  server.on("/execute",  HTTP_POST, handleExecute);
  server.on("/sequence", HTTP_POST, handleSequence);
  server.on("/health",   HTTP_GET,  handleHealth);
  server.onNotFound(handleNotFound);
  server.begin();
  MDNS.addService("http", "tcp", cfg::HTTP_LISTEN_PORT);
  Serial.printf("✅ HTTP server on :%u (/execute, /sequence, /health)\n",
                cfg::HTTP_LISTEN_PORT);
}

void loop() {
  feedWDT();
  if (WiFi.status() != WL_CONNECTED) { WiFi.reconnect(); delay(500); return; }

  server.handleClient();  // 處理 /execute /health
  tickMotion();           // 每 tick 更新一次 PWM（若有 active motion）
  tickExecute();          // 推進階段/PID FSM
  tickIdle();             // 沒事就掃 vision，有目標就啟動
}
