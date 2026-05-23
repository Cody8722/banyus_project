#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>

struct Block {
  int signature, x, y, width, height;
};

const char* ssid       = "1234567";
const char* pass       = "asdfghjk";
const char* server_url = "http://10.20.171.77:5000/arm";

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define PIXY_ADDR 0x54

unsigned long lastVision  = 0;
const int VISION_INTERVAL = 3000;
bool busy = false;

int cur[4] = {90, 90, 120, 5};

int angleToPWM(int angle) {
  return map(angle, 0, 180, 150, 600);
}

// ── S-curve 平滑運動（取代原 moveTrap） ───────────
// 5 階 smoothstep s(t)=t³(6t²−15t+10)：起終點 v 與 a 皆為 0，jerk-limited
void moveSmooth(int ch0, int ch1, int ch2, int ch3, int total_ms = 1200) {
  int target[4] = {ch0, ch1, ch2, ch3};
  const int steps = 40;

  for (int s = 1; s <= steps; s++) {
    float t = (float)s / steps;
    float ratio = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);

    for (int ch = 0; ch < 4; ch++) {
      int angle = cur[ch] + (int)((target[ch] - cur[ch]) * ratio);
      pwm.setPWM(ch, 0, angleToPWM(constrain(angle, 0, 175)));
    }
    esp_task_wdt_reset();
    delay(total_ms / steps);
  }
  for (int ch = 0; ch < 4; ch++) cur[ch] = target[ch];
}

// ── Pixy2 I2C 讀取 ────────────────────────────────
Block getBlockBySig(int sig) {
  Block b = {0, -1, -1, 0, 0};
  Wire1.beginTransmission(PIXY_ADDR);
  Wire1.write(0x20); Wire1.write(0x00); Wire1.write(0x04);
  Wire1.endTransmission();
  delay(5);

  int count = 0;
  Wire1.requestFrom(PIXY_ADDR, 2);
  if (Wire1.available() >= 2) {
    Wire1.read(); count = Wire1.read();
  }

  for (int i = 0; i < count; i++) {
    Wire1.requestFrom(PIXY_ADDR, 14);
    if (Wire1.available() >= 14) {
      Wire1.read(); Wire1.read(); Wire1.read(); Wire1.read();
      int s = Wire1.read() | (Wire1.read() << 8);
      int x = Wire1.read() | (Wire1.read() << 8);
      int y = Wire1.read() | (Wire1.read() << 8);
      int w = Wire1.read() | (Wire1.read() << 8);
      int h = Wire1.read() | (Wire1.read() << 8);
      if (s == sig) { b = {s, x, y, w, h}; return b; }
    }
  }
  return b;
}

// ── 完整 PID 視覺修正（取代原 P-only visualPID） ──
// 對準目標物像素 u，僅修正 ch0；KP 為原值 0.4×1.075≈0.43，加 KI/KD
int visualPID(int target_u, int max_iter = 3) {
  const float Kp    = 0.43f;
  const float Ki    = 0.05f;
  const float Kd    = 0.10f;
  const float I_MAX = 30.0f;

  int   ch0      = cur[0];
  float integral = 0.0f;
  int   prev_err = 0;

  for (int i = 0; i < max_iter; i++) {
    esp_task_wdt_reset();
    delay(300);  // 等馬達穩定

    Block b = getBlockBySig(1);
    if (b.x == -1) break;

    int error = target_u - b.x;
    if (abs(error) < 5) break;  // 收斂

    integral = constrain(integral + (float)error, -I_MAX, I_MAX);
    float correction = Kp * error + Ki * integral + Kd * (error - prev_err);
    prev_err = error;

    int delta = (int)lroundf(correction);
    if (delta == 0) break;
    ch0 = constrain(ch0 + delta, 5, 175);

    Serial.printf("  PID iter=%d err=%d i=%.1f Δ=%d ch0=%d\n",
                  i, error, integral, delta, ch0);

    moveSmooth(ch0, cur[1], cur[2], cur[3], 400);
  }
  return ch0;
}

// ── 執行動作序列 ──────────────────────────────────
void executeAngles(DynamicJsonDocument& doc, int target_u) {
  const char* phases[] = {"pick", "grip", "place", "release", "home"};

  for (int i = 0; i < 5; i++) {
    if (!doc.containsKey(phases[i])) continue;

    JsonObject p = doc[phases[i]];
    int ch0 = constrain((int)p["ch0"], 5, 175);
    int ch1 = constrain((int)p["ch1"], 5, 175);
    int ch2 = constrain((int)p["ch2"], 5, 175);
    int ch3 = constrain((int)p["ch3"], 5, 90);

    Serial.printf("▶ %s → ch0=%d ch1=%d ch2=%d ch3=%d\n",
                  phases[i], ch0, ch1, ch2, ch3);

    moveSmooth(ch0, ch1, ch2, ch3);
    delay(300);
    esp_task_wdt_reset();

    // pick 之後做 PID 對準
    if (strcmp(phases[i], "pick") == 0 && target_u > 0) {
      Serial.println("  🎯 視覺 PID 修正中...");
      visualPID(target_u);
    }
  }
}

void sendAndExecute(int tx, int ty, int dx, int dy) {
  HTTPClient http;
  http.begin(server_url);
  http.addHeader("Content-Type", "application/json");

  String body = "{\"tx\":" + String(tx) + ",\"ty\":" + String(ty) +
                ",\"dx\":" + String(dx) + ",\"dy\":" + String(dy) + "}";

  Serial.printf("📤 POST: %s\n", body.c_str());
  int code = http.POST(body);

  if (code == 200) {
    String resp = http.getString();
    Serial.printf("✅ 收到角度\n");
    DynamicJsonDocument doc(2048);
    deserializeJson(doc, resp);
    executeAngles(doc, tx);
    Serial.println("✅ 動作完成");
  } else {
    Serial.printf("❌ HTTP 失敗 code=%d\n", code);
  }

  http.end();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🚀 啟動中...");

  esp_task_wdt_deinit();
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 30000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(50);

  moveSmooth(90, 90, 120, 5, 2000);
  Serial.println("✅ 伺服馬達初始化完成");

  Wire1.begin(4, 5);
  Serial.println("✅ Pixy2 I2C 初始化完成");

  WiFi.begin(ssid, pass);
  Serial.print("WiFi 連線中");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 40) {
    esp_task_wdt_reset(); delay(500); Serial.print("."); retries++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n❌ WiFi 失敗，重啟");
    ESP.restart();
  }
  Serial.println("\n✅ WiFi 連線成功");
  Serial.println(WiFi.localIP());
}

void loop() {
  esp_task_wdt_reset();

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    delay(500);
    return;
  }

  if (!busy && millis() - lastVision >= VISION_INTERVAL) {
    lastVision = millis();

    Block target = getBlockBySig(1);
    Block dest   = getBlockBySig(2);

    if (target.x != -1 && dest.x != -1) {
      Serial.printf("👁️ 目標:(%d,%d) 目的地:(%d,%d)\n",
                    target.x, target.y, dest.x, dest.y);
      busy = true;
      sendAndExecute(target.x, target.y, dest.x, dest.y);
      busy = false;
      lastVision = millis();
    } else {
      if (target.x == -1) Serial.println("👁️ 找不到目標物");
      if (dest.x   == -1) Serial.println("👁️ 找不到放置目的地");
    }
  }
}
