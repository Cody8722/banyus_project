// Robotic arm controller — ESP32 + PCA9685 + Pixy2 + HTTP backend
// 保留原本功能，優化：梯形速度連續性、HTTP 重試、記憶體、常數抽出。

#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>

// ── 設定常數 ──────────────────────────────────────
namespace cfg {
  // WiFi / Server
  constexpr char SSID[]       = "1234567";
  constexpr char PASS[]       = "asdfghjk";
  constexpr char SERVER_URL[] = "http://10.20.171.77:5000/arm";
  constexpr uint16_t HTTP_TIMEOUT_MS = 5000;
  constexpr uint8_t  HTTP_MAX_RETRY  = 2;

  // I2C
  constexpr uint8_t  PWM_ADDR   = 0x40;
  constexpr uint8_t  PIXY_ADDR  = 0x54;
  constexpr uint8_t  SDA_PWM    = 21;
  constexpr uint8_t  SCL_PWM    = 22;
  constexpr uint8_t  SDA_PIXY   = 4;
  constexpr uint8_t  SCL_PIXY   = 5;

  // Servo
  constexpr uint16_t PWM_FREQ   = 50;
  constexpr uint16_t PWM_MIN    = 150;   // 對應 0°
  constexpr uint16_t PWM_MAX    = 600;   // 對應 180°
  constexpr uint8_t  SERVO_LO   = 5;
  constexpr uint8_t  SERVO_HI   = 175;
  constexpr uint8_t  GRIPPER_HI = 90;

  // Home position
  constexpr uint8_t  HOME_CH0   = 90;
  constexpr uint8_t  HOME_CH1   = 90;
  constexpr uint8_t  HOME_CH2   = 120;
  constexpr uint8_t  HOME_CH3   = 5;

  // Motion
  constexpr uint16_t MOVE_STEPS   = 40;
  constexpr uint16_t MOVE_MS      = 1200;
  constexpr uint16_t MOVE_FAST_MS = 400;
  constexpr uint16_t MOVE_INIT_MS = 2000;
  constexpr float    ACCEL_FRAC   = 0.25f;   // 加減速段佔比

  // Vision
  constexpr uint16_t VISION_INTERVAL_MS = 3000;
  constexpr uint8_t  PIXY_SIG_TARGET    = 1;
  constexpr uint8_t  PIXY_SIG_DEST      = 2;
  constexpr uint8_t  PIXY_MAX_BLOCKS    = 4;
  constexpr uint8_t  PIXY_READ_DELAY_MS = 5;

  // P 控制器
  constexpr float    KP            = 0.43f;   // 原 0.4 × 1.075（已合併單位換算）
  constexpr uint8_t  PID_MAX_ITER  = 3;
  constexpr uint8_t  PID_TOLERANCE = 5;        // 像素
  constexpr uint16_t PID_SETTLE_MS = 300;

  // Watchdog
  constexpr uint32_t WDT_TIMEOUT_MS = 30000;

  // JSON
  constexpr size_t   JSON_BUF_SIZE = 2048;
}

// ── 型別 ──────────────────────────────────────────
struct Block {
  int signature = 0;
  int x = -1;
  int y = -1;
  int width = 0;
  int height = 0;
  bool valid() const { return x >= 0; }
};

// ── 全域物件 / 狀態 ───────────────────────────────
static Adafruit_PWMServoDriver pwm(cfg::PWM_ADDR);
static int           cur[4]     = {cfg::HOME_CH0, cfg::HOME_CH1, cfg::HOME_CH2, cfg::HOME_CH3};
static bool          busy       = false;
static unsigned long lastVision = 0;

// ── 工具 ──────────────────────────────────────────
static inline int  angleToPWM(int a) { return map(a, 0, 180, cfg::PWM_MIN, cfg::PWM_MAX); }
static inline void feedWDT()         { esp_task_wdt_reset(); }

// ── 平滑梯形速度曲線 ─────────────────────────────
// 三段：加速 / 等速 / 減速。位置曲線在段邊界一階可微，避免速度跳變。
// v 為等速段速度，選擇 v = 1/(1−ta) 使總距離正好 1。
static float trapezoidalRatio(float t) {
  const float ta = cfg::ACCEL_FRAC;
  const float v  = 1.0f / (1.0f - ta);
  if (t < ta) {
    return 0.5f * v * t * t / ta;                      // 0 → v·ta/2
  }
  if (t > 1.0f - ta) {
    const float dt = 1.0f - t;
    return 1.0f - 0.5f * v * dt * dt / ta;             // (1 − v·ta/2) → 1
  }
  return 0.5f * v * ta + v * (t - ta);                 // 等速
}

static void moveTrap(int ch0, int ch1, int ch2, int ch3, int total_ms = cfg::MOVE_MS) {
  const int target[4] = {ch0, ch1, ch2, ch3};
  const int steps     = cfg::MOVE_STEPS;
  const int dt_ms     = max(1, total_ms / steps);

  for (int s = 1; s <= steps; s++) {
    const float r = trapezoidalRatio((float)s / steps);
    for (int ch = 0; ch < 4; ch++) {
      const int angle = cur[ch] + (int)((target[ch] - cur[ch]) * r);
      pwm.setPWM(ch, 0, angleToPWM(constrain(angle, 0, cfg::SERVO_HI)));
    }
    feedWDT();
    delay(dt_ms);
  }
  memcpy(cur, target, sizeof(cur));
}

// ── Pixy2 I2C 讀取（保留原協定） ──────────────────
static Block getBlockBySig(int sig) {
  Block b;
  Wire1.beginTransmission(cfg::PIXY_ADDR);
  Wire1.write(0x20);
  Wire1.write(0x00);
  Wire1.write(cfg::PIXY_MAX_BLOCKS);
  Wire1.endTransmission();
  delay(cfg::PIXY_READ_DELAY_MS);

  int count = 0;
  Wire1.requestFrom(cfg::PIXY_ADDR, (uint8_t)2);
  if (Wire1.available() >= 2) {
    Wire1.read();           // skip
    count = Wire1.read();
  }
  if (count <= 0) return b;

  for (int i = 0; i < count; i++) {
    if (Wire1.requestFrom(cfg::PIXY_ADDR, (uint8_t)14) < 14) continue;
    if (Wire1.available() < 14) continue;
    for (int k = 0; k < 4; k++) Wire1.read();          // skip header
    const int s = Wire1.read() | (Wire1.read() << 8);
    const int x = Wire1.read() | (Wire1.read() << 8);
    const int y = Wire1.read() | (Wire1.read() << 8);
    const int w = Wire1.read() | (Wire1.read() << 8);
    const int h = Wire1.read() | (Wire1.read() << 8);
    if (s == sig) { b = {s, x, y, w, h}; return b; }
  }
  return b;
}

// ── 視覺 P 控制（只修正 ch0） ─────────────────────
static void visualAlign(int target_u) {
  int ch0 = cur[0];
  for (uint8_t i = 0; i < cfg::PID_MAX_ITER; i++) {
    feedWDT();
    delay(cfg::PID_SETTLE_MS);

    const Block b = getBlockBySig(cfg::PIXY_SIG_TARGET);
    if (!b.valid()) break;

    const int err = target_u - b.x;
    if (abs(err) < cfg::PID_TOLERANCE) break;

    const int delta = (int)lroundf(cfg::KP * err);
    if (delta == 0) break;
    ch0 = constrain(ch0 + delta, cfg::SERVO_LO, cfg::SERVO_HI);

    Serial.printf("  PID iter=%u err=%d delta=%d ch0=%d\n", i, err, delta, ch0);
    moveTrap(ch0, cur[1], cur[2], cur[3], cfg::MOVE_FAST_MS);
  }
}

// ── 從 JSON 階段物件取出 4 軸角度 ─────────────────
static bool extractPhase(JsonVariant p, int out[4]) {
  if (!p.is<JsonObject>()) return false;
  out[0] = constrain((int)p["ch0"], cfg::SERVO_LO, cfg::SERVO_HI);
  out[1] = constrain((int)p["ch1"], cfg::SERVO_LO, cfg::SERVO_HI);
  out[2] = constrain((int)p["ch2"], cfg::SERVO_LO, cfg::SERVO_HI);
  out[3] = constrain((int)p["ch3"], cfg::SERVO_LO, cfg::GRIPPER_HI);
  return true;
}

// ── 執行五階段動作序列 ────────────────────────────
static void executeAngles(DynamicJsonDocument& doc, int target_u) {
  static const char* const phases[] = {"pick", "grip", "place", "release", "home"};
  int a[4];

  for (const char* phase : phases) {
    if (!extractPhase(doc[phase], a)) continue;

    Serial.printf("▶ %s → ch0=%d ch1=%d ch2=%d ch3=%d\n",
                  phase, a[0], a[1], a[2], a[3]);

    moveTrap(a[0], a[1], a[2], a[3]);
    delay(300);
    feedWDT();

    if (target_u > 0 && strcmp(phase, "pick") == 0) {
      Serial.println("  🎯 視覺修正中...");
      visualAlign(target_u);
      delay(200);
      feedWDT();
    }
  }
}

// ── 與伺服器溝通並執行 ────────────────────────────
static void sendAndExecute(int tx, int ty, int dx, int dy) {
  char body[96];
  const int n = snprintf(body, sizeof(body),
                         "{\"tx\":%d,\"ty\":%d,\"dx\":%d,\"dy\":%d}",
                         tx, ty, dx, dy);
  if (n <= 0 || n >= (int)sizeof(body)) {
    Serial.println("❌ body 編碼失敗");
    return;
  }
  Serial.printf("📤 POST: %s\n", body);

  HTTPClient http;
  http.setTimeout(cfg::HTTP_TIMEOUT_MS);
  http.setReuse(false);

  int code = -1;
  String resp;
  for (uint8_t attempt = 0; attempt <= cfg::HTTP_MAX_RETRY; attempt++) {
    if (!http.begin(cfg::SERVER_URL)) {
      Serial.println("❌ HTTP begin 失敗");
      delay(200);
      continue;
    }
    http.addHeader("Content-Type", "application/json");
    code = http.POST((uint8_t*)body, n);
    if (code == 200) {
      resp = http.getString();
      http.end();
      break;
    }
    Serial.printf("⚠ HTTP code=%d (重試 %u/%u)\n",
                  code, attempt + 1, cfg::HTTP_MAX_RETRY);
    http.end();
    feedWDT();
    delay(300);
  }

  if (code != 200) {
    Serial.printf("❌ HTTP 失敗 code=%d\n", code);
    return;
  }

  Serial.println("✅ 收到角度");
  DynamicJsonDocument doc(cfg::JSON_BUF_SIZE);
  const DeserializationError err = deserializeJson(doc, resp);
  if (err) {
    Serial.printf("❌ JSON 解析失敗: %s\n", err.c_str());
    return;
  }
  executeAngles(doc, tx);
  Serial.println("✅ 動作完成");
}

// ── WiFi ──────────────────────────────────────────
static bool wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg::SSID, cfg::PASS);
  Serial.print("WiFi 連線中");
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) {
    feedWDT();
    delay(500);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi 連線失敗");
    return false;
  }
  Serial.printf("✅ WiFi 連線成功 %s\n", WiFi.localIP().toString().c_str());
  return true;
}

// ── 看門狗 ────────────────────────────────────────
static void initWatchdog() {
  esp_task_wdt_deinit();
  esp_task_wdt_config_t wdt = {
    .timeout_ms     = cfg::WDT_TIMEOUT_MS,
    .idle_core_mask = 0,
    .trigger_panic  = true,
  };
  esp_task_wdt_init(&wdt);
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
  moveTrap(cfg::HOME_CH0, cfg::HOME_CH1, cfg::HOME_CH2, cfg::HOME_CH3,
           cfg::MOVE_INIT_MS);
  Serial.println("✅ 伺服馬達初始化完成");

  Wire1.begin(cfg::SDA_PIXY, cfg::SCL_PIXY);
  Serial.println("✅ Pixy2 I2C 初始化完成");

  if (!wifiConnect()) ESP.restart();
}

void loop() {
  feedWDT();

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    delay(500);
    return;
  }

  if (busy || millis() - lastVision < cfg::VISION_INTERVAL_MS) return;
  lastVision = millis();

  const Block target = getBlockBySig(cfg::PIXY_SIG_TARGET);
  const Block dest   = getBlockBySig(cfg::PIXY_SIG_DEST);

  if (!target.valid()) { Serial.println("👁 找不到目標物"); return; }
  if (!dest.valid())   { Serial.println("👁 找不到放置目的地"); return; }

  Serial.printf("👁 目標:(%d,%d) 目的地:(%d,%d)\n",
                target.x, target.y, dest.x, dest.y);
  busy = true;
  sendAndExecute(target.x, target.y, dest.x, dest.y);
  busy = false;
  lastVision = millis();
}
