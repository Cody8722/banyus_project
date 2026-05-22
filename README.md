# Banyus Arm — ESP32 視覺機械手臂控制器

ESP32 主控：Pixy2 視覺定位 → HTTP 後端計算逆運動學 → PCA9685 驅動 4 軸伺服馬達執行夾取動作。內建 S-curve 平滑運動、多幀中位數視覺去雜訊、完整 PID 視覺修正。

---

## 硬體

| 元件 | 用途 | I2C 位址 |
|---|---|---|
| ESP32 (WROOM / WROVER) | 主控 | — |
| PCA9685 16-ch PWM 驅動板 | 伺服 PWM 訊號 | `0x40` |
| Pixy2 視覺感測器 | 偵測目標物與目的地 | `0x54` |
| 4 × 伺服馬達 (MG996R / SG90 等) | 4 軸機構 | — |
| 5V 3A+ 電源 | 伺服電源（與 ESP32 共地） | — |
| HTTP 後端伺服器 | 接收 tx/ty/dx/dy 計算角度 | — |

## 接線

```
ESP32          PCA9685         Pixy2
─────          ───────         ─────
GPIO 21  ───── SDA
GPIO 22  ───── SCL
GPIO  4  ────────────────────  SDA
GPIO  5  ────────────────────  SCL
GND      ───── GND      ─────  GND
3V3      ───── VCC      ─────  VCC

PCA9685 V+  ←─  5V 外接電源（伺服用）
PCA9685 CH0 ←─  底座旋轉伺服
PCA9685 CH1 ←─  肩（大臂）伺服
PCA9685 CH2 ←─  肘（小臂）伺服
PCA9685 CH3 ←─  夾爪伺服
```

> ⚠ 伺服電源請用獨立 5V，**不要**從 ESP32 取電。PCA9685 V+ 與 ESP32 共 GND。

---

## 軟體依賴

Arduino IDE 或 PlatformIO，安裝：

- ESP32 board support（ESP-IDF v5 以上，本程式用 `esp_task_wdt_config_t` 新 API）
- `Adafruit_PWMServoDriver`
- `ArduinoJson` v6.x

開啟 `banyus_arm/banyus_arm.ino`，選好板子，編譯上傳。

---

## 參數參考

所有可調參數都集中在 `banyus_arm.ino` 頂部的 `namespace cfg`。下表列出每個常數的用途、預設值、單位、合理範圍與調整方向。

### WiFi / 伺服器

| 參數 | 預設 | 說明 / 調整方向 |
|---|---|---|
| `SSID` | `"1234567"` | WiFi 網路名稱 |
| `PASS` | `"asdfghjk"` | WiFi 密碼 |
| `SERVER_URL` | `"http://10.20.171.77:5000/arm"` | 後端 HTTP 端點（含 path） |
| `HTTP_TIMEOUT_MS` | `5000` ms | 單次 HTTP 請求逾時。網路差就拉長 |
| `HTTP_MAX_RETRY` | `2` | 失敗重試次數，實際嘗試 N+1 次 |

### I2C 腳位 / 位址

| 參數 | 預設 | 說明 |
|---|---|---|
| `PWM_ADDR` | `0x40` | PCA9685 I2C 位址（板上 A0–A5 跳線可改） |
| `PIXY_ADDR` | `0x54` | Pixy2 I2C 位址（PixyMon 設定中可改） |
| `SDA_PWM` / `SCL_PWM` | `21` / `22` | PCA9685 用 `Wire`（預設 I2C 通道） |
| `SDA_PIXY` / `SCL_PIXY` | `4` / `5` | Pixy2 用 `Wire1`（第二 I2C 通道） |

### 伺服馬達

| 參數 | 預設 | 單位 | 說明 |
|---|---|---|---|
| `PWM_FREQ` | `50` | Hz | PWM 頻率。類比伺服 50Hz，數位伺服可到 333Hz |
| `PWM_MIN` | `150` | tick | 對應 0° 的 PWM 值（PCA9685 為 12-bit, 0–4095）。≈ 500µs 脈寬 |
| `PWM_MAX` | `600` | tick | 對應 180° 的 PWM 值。≈ 2500µs 脈寬 |
| `SERVO_LO` | `5` | ° | 各軸最小允許角度（避免撞限位） |
| `SERVO_HI` | `175` | ° | 各軸最大允許角度 |
| `GRIPPER_HI` | `90` | ° | 夾爪（ch3）最大角度，依機構決定方向 |

> **校正 PWM_MIN/MAX**：先設 150/600，下命令到 0° 和 180°，量實際角度，按比例修正。不同廠牌伺服可差 ±15%。

### Home 位置（開機歸位的安全姿勢）

| 參數 | 預設 | 說明 |
|---|---|---|
| `HOME_CH0` | `90` | 底座旋轉，置中 |
| `HOME_CH1` | `90` | 大臂（肩），中位 |
| `HOME_CH2` | `120` | 小臂（肘），稍微抬起避免撞地 |
| `HOME_CH3` | `5` | 夾爪，閉合狀態 |

### 動作時間

| 參數 | 預設 | 單位 | 說明 |
|---|---|---|---|
| `MOVE_STEPS` | `40` | step | 一次移動分幾步寫 PWM。↑ 越平滑但 CPU 用量更高 |
| `MOVE_MS` | `1200` | ms | 一般階段動作時間。↓ 動作快但易抖；↑ 慢但穩 |
| `MOVE_FAST_MS` | `400` | ms | PID 小幅修正動作時間 |
| `MOVE_INIT_MS` | `2000` | ms | 開機歸位時間。較長以保安全 |
| `PHASE_SETTLE_MS` | `300` | ms | 每階段動作完成後等待，讓伺服真正穩定 |

> 動作解析度 = `MOVE_MS / MOVE_STEPS` = 30ms/step。`tickMotion` 中若同一 step 重複進入會略過，不會多餘寫 I2C。

### 視覺

| 參數 | 預設 | 單位 | 說明 |
|---|---|---|---|
| `VISION_INTERVAL_MS` | `3000` | ms | IDLE 狀態下兩次掃描的最短間隔 |
| `PIXY_SIG_TARGET` | `1` | — | Pixy2 中標記目標物的 signature 編號 |
| `PIXY_SIG_DEST` | `2` | — | 標記放置位置的 signature 編號 |
| `PIXY_MAX_BLOCKS` | `4` | block | 一次 I2C 請求要拉的最大 block 數 |
| `PIXY_READ_DELAY_MS` | `5` | ms | Pixy2 對請求的反應延遲 |
| `MEDIAN_SAMPLES` | `3` | frame | 中位數樣本數（建議奇數 3 / 5 / 7） |
| `MEDIAN_GAP_MS` | `20` | ms | 中位數樣本間的等待 |

### PID（視覺對準，僅作用於 ch0）

| 參數 | 預設 | 說明 |
|---|---|---|
| `KP` | `0.43` | 比例增益。每 1 px 誤差 → 修正角度數。↑ 反應快但易震盪 |
| `KI` | `0.05` | 積分增益。消除穩態誤差。↑ 太多會 windup |
| `KD` | `0.10` | 微分增益。抑制超調。↑ 太多對雜訊敏感 |
| `I_MAX` | `30` | 積分項夾鉗上下限，防止 windup |
| `PID_MAX_ITER` | `3` | 最多修正次數，避免無限循環 |
| `PID_TOLERANCE` | `5` | 誤差 < 此值（像素）視為達成 |
| `PID_SETTLE_MS` | `300` | ms。每次 iter 等待伺服穩定 |

> **調校順序**：先 KI = KD = 0，調 KP 到剛好不震盪 → 加 KD 抑制超調 → 最後加 KI 消除穩態誤差。

### 系統 / 緩衝

| 參數 | 預設 | 說明 |
|---|---|---|
| `WDT_TIMEOUT_MS` | `30000` ms | Task Watchdog 逾時。任一操作（含 HTTP）超過就 panic 重啟 |
| `JSON_BUF_SIZE` | `2048` B | JSON 解析緩衝。5 階段 × ~80B/階段 ≈ 400B，2KB 已有充裕餘裕 |
| `PCA_LED0_ON_L` | `0x06` | PCA9685 LED0_ON_L 暫存器位址，批次寫入起點 |

### 推算公式速查

```
PWM_value     = map(angle, 0, 180, PWM_MIN, PWM_MAX)
pulse_width   = PWM_value / 4096 × (1000 / PWM_FREQ)  ms
step_duration = MOVE_MS / MOVE_STEPS                  ms
PID_total_ms  = PID_MAX_ITER × (PID_SETTLE_MS + MOVE_FAST_MS)
```

---

## 系統架構

### 主狀態機

```
┌─────────┐    達到 vision interval    ┌──────────┐
│  IDLE   │ ──────────────────────────►│ EXECUTE  │
│         │                             │          │
│ 等視覺   │                             │ 跑 5 階段 │
│ 間隔    │ ◄───────────────────────────│ 序列     │
└─────────┘       全部階段完成           └──────────┘
```

### EXECUTE 子狀態

```
              ┌────────────┐
   進入  ──►  │ PHASE_MOVE │   等待 motion 完成
              └─────┬──────┘
                    ▼
              ┌──────────────┐
              │ PHASE_SETTLE │   PHASE_SETTLE_MS
              └─────┬────────┘
                    │
            是 pick + target_u > 0 ?
              ┌─────┴─────┐
            否│           │是
              │           ▼
              │     ┌──────────────┐
              │     │ PID_SETTLE   │ ◄─────┐
              │     └─────┬────────┘       │
              │           │                │
              │     達 max_iter / 誤差OK    │
              │     ┌─────┴─────┐          │
              │     │           │          │
              │     │           ▼          │
              │     │     ┌──────────┐     │
              │     │     │ PID_MOVE │     │
              │     │     └─────┬────┘     │
              │     │           └──────────┘
              ▼     ▼
        ┌────────────────┐
        │ 下一階段 / 完成 │
        └────────────────┘
```

5 個階段名稱（依序執行）：`pick` → `grip` → `place` → `release` → `home`
JSON 中省略的階段會被跳過。

### 動作曲線：S-curve（5 階 smoothstep）

`s(t) = t³(6t² − 15t + 10)`

```
位置 ratio
1.0 ┤                       ╭────
    │                  ╱
0.5 ┤             ╱
    │        ╱
0.0 ┤────╱
    └─────────────────────────── 時間 t
    0                            1
```

特性：起點與終點的速度與加速度皆為 0（jerk-limited），對伺服減速箱與機構更友善。

### 視覺去雜訊

```
read frame 1 ─┐
read frame 2 ─┼─► median(x, y, w, h) → Block
read frame 3 ─┘
```

單幀 Pixy 讀取可能受光源閃爍、邊緣抖動干擾。取奇數張中位數可過濾離群值。

### PID 視覺修正（僅作用於 ch0）

```
                        ┌──── KP ────┐
                        │            │
err = target_u − pixel_x ─┼──── KI·∫ ──┼─► Δch0 ─► constrain ─► moveTrap
                        │            │
                        └─KD·Δerr ──┘
```

積分項在 `±I_MAX` 內夾鉗以防 windup。

---

## HTTP 協定

### Request

```http
POST /arm HTTP/1.1
Content-Type: application/json

{ "tx": 145, "ty": 88, "dx": 200, "dy": 120 }
```

| 欄位 | 型別 | 意義 |
|---|---|---|
| `tx`, `ty` | int | 目標物像素座標 (Pixy 視野內) |
| `dx`, `dy` | int | 放置目的地像素座標 |

### Response

`200 OK` + JSON：

```json
{
  "pick":    { "ch0": 80,  "ch1": 100, "ch2": 130, "ch3": 5  },
  "grip":    { "ch0": 80,  "ch1": 100, "ch2": 130, "ch3": 60 },
  "place":   { "ch0": 110, "ch1": 95,  "ch2": 125, "ch3": 60 },
  "release": { "ch0": 110, "ch1": 95,  "ch2": 125, "ch3": 5  },
  "home":    { "ch0": 90,  "ch1": 90,  "ch2": 120, "ch3": 5  }
}
```

- 缺漏的階段會被跳過
- 角度會被 `constrain` 到 `SERVO_LO~SERVO_HI`（ch3 上限為 `GRIPPER_HI`）

### 重試

HTTP 失敗時自動重試 2 次，間隔指數退避：`300ms → 600ms → 1200ms`（上限 2000ms）。

---

## 動作流程

1. 上電 → `setup()` 歸位到 HOME（2 秒）
2. 連 WiFi（最多 20 秒，失敗則重啟）
3. 進入 `loop()`：
   - 每 3 秒（`VISION_INTERVAL_MS`）掃描 Pixy 取 sig1/sig2 中位數
   - 兩者都找到 → POST 給伺服器
   - 收到角度 → 依序執行 pick / grip / place / release / home
   - pick 階段完成後做視覺 PID 修正（最多 3 次）
   - home 完成 → 回 IDLE

---

## 參數調校

| 症狀 | 調整 |
|---|---|
| 動作太快、伺服顫動 | `MOVE_MS` ↑（例如 1500） |
| 動作太慢 | `MOVE_MS` ↓ |
| PID 反應遲鈍 | `KP` ↑ |
| PID 抖動 / 超調 | `KP` ↓ 或 `KD` ↑ |
| PID 有穩態誤差 | `KI` ↑（小心積分飽和） |
| 積分發散 | `I_MAX` ↓ |
| 視覺誤觸發 | `MEDIAN_SAMPLES` ↑ (5 或 7) |
| Pixy 找不到目標 | 檢查光源、重新訓練 signature |
| 開機歸位太猛 | `MOVE_INIT_MS` ↑ |

> PID 調校建議：先把 KI、KD 設 0，調 KP 到剛好不抖動；再加 KD 抑制超調；最後加 KI 消除穩態誤差。

---

## 疑難排解

| 訊息 / 現象 | 可能原因 |
|---|---|
| `❌ WiFi 連線失敗` | SSID/密碼錯、訊號弱、路由器問題（系統會重啟） |
| `⚠ HTTP code=-1` | 連不到伺服器（IP/Port、防火牆、後端未啟動） |
| `⚠ HTTP code=500` | 後端錯誤，看伺服器日誌 |
| `❌ JSON 解析失敗` | 後端回傳非 JSON 或格式異常 |
| `👁 找不到目標物` | Pixy2 視野中未偵測到對應 signature |
| Watchdog 觸發重啟 | 某操作 > 30 秒（多半 I2C 卡死），檢查接線與電源 |
| 伺服角度不對 | 校正 `PWM_MIN` / `PWM_MAX`（不同伺服廠牌有差） |
| 動作中段速度過快 | S-curve 中段較快，`MOVE_MS` ↑ 或改回梯形 |

---

## 檔案結構

```
banyus_project/
├── README.md
└── banyus_arm/
    └── banyus_arm.ino
```

## 修改紀錄

| Commit | 重點 |
|---|---|
| `bfdab44` | 非阻塞 FSM、S-curve、中位數視覺、完整 PID、PCA 批次寫、串流 JSON |
| `a1f0e32` | 初版優化：梯形平滑、HTTP 重試、常數抽出 |
