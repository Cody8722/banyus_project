"""Banyus arm — Flask backend (polling architecture).

ESP32 是純 HTTP server，暴露:
  - GET  /vision   → Pixy 中位數座標 {target_found, dest_found, tx, ty, dx, dy}
  - POST /execute  → 收 5 階段 JSON（可含 target_u 觸發 PID）執行動作

本伺服器:
  - 定期 GET /vision（AUTO 模式開啟時）
  - 偵測到 target+dest → 算 IK → POST /execute
  - 終端可手動 scan / 推單一姿勢

IK 公式（cal_arm_angle / angles_to_servo / 內插表）原樣保留。
"""
from __future__ import annotations

import json
import logging
import math
import threading
from typing import Optional

import cv2
import numpy as np
import requests
from flask import Flask, jsonify, request

# ── 設定 ─────────────────────────────────────────
HOST = "0.0.0.0"
PORT = 5000
ESP32_URL     = "http://banyus.local"   # mDNS 名稱，DHCP 變動也免改
ESP32_TIMEOUT = 30                       # /execute 包含整段動作時間

POLL_INTERVAL_S = 3.0                    # AUTO 模式輪詢間隔
VISION_TIMEOUT  = 5                      # GET /vision 逾時

OBJECT_Z_MM      = 10
PLACE_CH0_OFFSET = -5

HOME_ANGLES  = {"ch0": 90, "ch1": 90,  "ch2": 120, "ch3": 5}
INITIAL_POSE = {"ch0": 90, "ch1": 153, "ch2": 25,  "ch3": 5}

CH3_OPEN   = 5
CH3_CLOSED = 90

SERVO_LIMITS = {
    "ch0": (5, 175),
    "ch1": (90, 175),
    "ch2": (0, 120),
    "ch3": (5, 90),
}

# 由實機校正得到的機構補償（不影響 IK 數學）
CH0_OFFSET_NEG = -4   # clamta < 0
CH0_OFFSET_POS = -5   # clamta > 0
CH1_OFFSET     = -3

# ── Logger ───────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("arm")
logging.getLogger("werkzeug").setLevel(logging.WARNING)

# ── Homography ───────────────────────────────────
PIXEL_PTS = np.array([
    [142, 151],
    [92,  138],
    [217, 119],
    [96,  202],
], dtype=np.float32)

REAL_PTS = np.array([
    [145,    0],
    [158,   50],
    [180,  -75],
    [95,    48],
], dtype=np.float32)

H, _ = cv2.findHomography(PIXEL_PTS, REAL_PTS)


def pixel_to_real(u: float, v: float) -> tuple[float, float]:
    pt = np.array([[[u, v]]], dtype=np.float32)
    real = cv2.perspectiveTransform(pt, H)
    return float(real[0][0][0]), float(real[0][0][1])


# ── IK 參數（保留） ──────────────────────────────
A1, A2, A3 = 81.4, 84.0, 67.0

CL_TABLE  = [13.6, 36.1, 49.2, 60.4, 71.1]
CH1_TABLE = [175,  160,  153,  140,  135]

CR_TABLE  = [2.9,  23.1, 33.7, 41.6, 47.6]
CH2_TABLE = [60,   40,   25,   10,   0]

LIMITS_RAD = {
    "clamta": (math.radians(-97),  math.radians(137)),
    "cL":     (math.radians(-15),  math.radians(128)),
    "beta":   (math.radians(25),   math.radians(160)),
    "cR":     (math.radians(-19.5), math.radians(116)),
}


# ── IK 計算（公式原樣保留） ──────────────────────
def cal_arm_angle(x: float, y: float, z: float) -> tuple[bool, float, float, float]:
    clamta = math.atan2(y, x)
    x2 = x - A3 * math.cos(clamta)
    y2 = y - A3 * math.sin(clamta)
    z2 = z
    D1 = math.sqrt(x2**2 + y2**2 + z2**2)

    cos_phi1 = (A1**2 + D1**2 - A2**2) / (2 * A1 * D1)
    cos_phi1 = max(-1.0, min(1.0, cos_phi1))
    phi1 = math.acos(cos_phi1)
    phi2 = math.atan2(z2, math.sqrt(x2**2 + y2**2))
    cL = phi1 + phi2

    cos_phi3 = (A1**2 + A2**2 - D1**2) / (2 * A1 * A2)
    cos_phi3 = max(-1.0, min(1.0, cos_phi3))
    phi3 = math.acos(cos_phi3)
    cR = math.pi - cL - phi3

    beta = cL + cR
    (cmin, cmax) = LIMITS_RAD["clamta"]
    (lmin, lmax) = LIMITS_RAD["cL"]
    (bmin, bmax) = LIMITS_RAD["beta"]
    (rmin, rmax) = LIMITS_RAD["cR"]
    isreachable = (
        cmin <= clamta <= cmax and
        lmin <= cL     <= lmax and
        bmin <= beta   <= bmax and
        rmin <= cR     <= rmax
    )
    return isreachable, clamta, cL, cR


def clamp(ch: str, value: int) -> int:
    lo, hi = SERVO_LIMITS[ch]
    return max(lo, min(hi, int(value)))


def angles_to_servo(clamta: float, cL: float, cR: float) -> dict:
    ch0 = 90 + math.degrees(clamta)
    if clamta < 0:
        ch0 += CH0_OFFSET_NEG
    elif clamta > 0:
        ch0 += CH0_OFFSET_POS

    ch1 = round(np.interp(math.degrees(cL), CL_TABLE, CH1_TABLE)) + CH1_OFFSET
    ch2 = round(np.interp(math.degrees(cR), CR_TABLE, CH2_TABLE))

    return {
        "ch0": clamp("ch0", ch0),
        "ch1": clamp("ch1", ch1),
        "ch2": clamp("ch2", ch2),
    }


def xyz_to_angles(x_mm: float, y_mm: float, z_mm: float) -> Optional[dict]:
    isreachable, clamta, cL, cR = cal_arm_angle(x_mm, y_mm, z_mm)
    if not isreachable:
        return None
    return angles_to_servo(clamta, cL, cR)


def build_phase(ik: dict, ch3: int, ch0_offset: int = 0) -> dict:
    return {
        "ch0": clamp("ch0", ik["ch0"] + ch0_offset),
        "ch1": ik["ch1"],
        "ch2": ik["ch2"],
        "ch3": ch3,
    }


def compute_sequence(tx_px: float, ty_px: float,
                     dx_px: float, dy_px: float) -> Optional[dict]:
    """像素座標 → 5 階段角度 + target_u（給 PID 用）"""
    tx, ty = pixel_to_real(tx_px, ty_px)
    dx, dy = pixel_to_real(dx_px, dy_px)
    target = xyz_to_angles(tx, ty, OBJECT_Z_MM)
    dest   = xyz_to_angles(dx, dy, OBJECT_Z_MM)
    if target is None or dest is None:
        return None
    return {
        "pick":     build_phase(target, CH3_OPEN),
        "grip":     build_phase(target, CH3_CLOSED),
        "place":    build_phase(dest,   CH3_CLOSED, ch0_offset=PLACE_CH0_OFFSET),
        "release":  build_phase(dest,   CH3_OPEN,   ch0_offset=PLACE_CH0_OFFSET),
        "home":     HOME_ANGLES,
        "target_u": int(tx_px),
    }


# ── 與 ESP32 溝通（共用 Session） ─────────────────
_session = requests.Session()


def get_vision() -> Optional[dict]:
    try:
        r = _session.get(f"{ESP32_URL}/vision", timeout=VISION_TIMEOUT)
        if r.status_code == 200:
            return r.json()
        if r.status_code == 503:
            return None  # busy，下次再試
        log.error("❌ /vision code=%d", r.status_code)
    except requests.RequestException as e:
        log.error("❌ /vision 失敗: %s", e)
    return None


def post_execute(payload: dict) -> bool:
    try:
        r = _session.post(f"{ESP32_URL}/execute", json=payload, timeout=ESP32_TIMEOUT)
        if r.status_code == 200:
            log.info("✅ /execute 已接受")
            return True
        if r.status_code == 503:
            log.warning("⚠ ESP32 忙碌中")
        else:
            log.error("❌ /execute code=%d body=%s", r.status_code, r.text)
    except requests.RequestException as e:
        log.error("❌ /execute 失敗: %s", e)
    return False


# ── AUTO 模式（輪詢 /vision） ─────────────────────
_state_lock = threading.Lock()
_stop_event = threading.Event()
AUTO_MODE   = False


def get_auto() -> bool:
    with _state_lock:
        return AUTO_MODE


def set_auto(value: bool) -> bool:
    global AUTO_MODE
    with _state_lock:
        AUTO_MODE = bool(value)
        return AUTO_MODE


def vision_poller() -> None:
    log.info("👁 視覺輪詢執行緒啟動 (interval=%.1fs)", POLL_INTERVAL_S)
    while not _stop_event.is_set():
        if not get_auto():
            _stop_event.wait(POLL_INTERVAL_S)
            continue

        vision = get_vision()
        if vision is None:
            _stop_event.wait(POLL_INTERVAL_S)
            continue

        if not (vision.get("target_found") and vision.get("dest_found")):
            log.info("👁 找不到目標 (target=%s dest=%s)",
                     vision.get("target_found"), vision.get("dest_found"))
            _stop_event.wait(POLL_INTERVAL_S)
            continue

        tx_px, ty_px = vision["tx"], vision["ty"]
        dx_px, dy_px = vision["dx"], vision["dy"]
        log.info("👁 target=(%d,%d) dest=(%d,%d)", tx_px, ty_px, dx_px, dy_px)

        seq = compute_sequence(tx_px, ty_px, dx_px, dy_px)
        if seq is None:
            log.warning("⚠ 超出可達範圍")
        else:
            log.info("📤 /execute ← %s", json.dumps(seq))
            post_execute(seq)

        # 動作中 ESP /vision 會回 503，自然形成節流
        _stop_event.wait(POLL_INTERVAL_S)


# ── Flask routes（保留 /arm 與 /health 供外部測試） ─
app = Flask(__name__)


@app.route("/arm", methods=["POST"])
def arm_control():
    """外部直接 POST 像素 → 回 5 階段角度，方便 curl 測 IK 管線（不會推 ESP32）。"""
    data = request.get_json(silent=True) or {}
    try:
        tx_px = float(data["tx"]); ty_px = float(data["ty"])
        dx_px = float(data["dx"]); dy_px = float(data["dy"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "missing or invalid tx/ty/dx/dy"}), 400

    log.info("📍 /arm target=(%.0f,%.0f) dest=(%.0f,%.0f)", tx_px, ty_px, dx_px, dy_px)
    seq = compute_sequence(tx_px, ty_px, dx_px, dy_px)
    if seq is None:
        log.warning("⚠ 超出可達範圍")
        return jsonify({"error": "out of range"}), 400
    return jsonify(seq)


@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok", "auto": get_auto()})


# ── 終端互動 ─────────────────────────────────────
class Terminal:
    HELP = (
        "\n🚀 手臂控制工具\n"
        "ESP32: {url}\n"
        "指令:\n"
        "  auto [on|off]  → 切換自動輪詢（/vision → IK → /execute）\n"
        "  scan           → 手動掃一次 /vision，找到就執行\n"
        "  status         → 顯示 AUTO 狀態與目前姿勢\n"
        "  x,y,z          → 真實座標(mm)\n"
        "  px=u,v         → 像素座標換算\n"
        "  ch0/1/2/3=角度 → 直接控制\n"
        "  all=ch0,ch1,ch2,ch3\n"
        "  home / q\n"
    )
    SINGLE_CH = ("ch0", "ch1", "ch2", "ch3")

    def __init__(self) -> None:
        self.pose: dict = dict(INITIAL_POSE)

    def run(self) -> None:
        print(self.HELP.format(url=ESP32_URL))
        while True:
            try:
                cmd = input("指令> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if cmd == "q":
                break
            if not cmd:
                continue
            if not self._dispatch(cmd):
                print("不認識的指令")
        _stop_event.set()
        post_execute({"home": HOME_ANGLES})
        print("✅ 結束")

    def _dispatch(self, cmd: str) -> bool:
        if cmd == "status":
            return self._cmd_status()
        if cmd == "scan":
            return self._cmd_scan()
        if cmd == "auto" or cmd.startswith("auto "):
            arg = cmd[5:].strip() if " " in cmd else ""
            return self._cmd_auto(arg)
        if cmd == "home":
            return self._cmd_home()
        if cmd.startswith("px="):
            return self._cmd_px(cmd[3:])
        if cmd.startswith("all="):
            return self._cmd_all(cmd[4:])
        for ch in self.SINGLE_CH:
            prefix = f"{ch}="
            if cmd.startswith(prefix):
                return self._cmd_single(ch, cmd[len(prefix):])
        if cmd.count(",") == 2:
            return self._cmd_xyz(cmd)
        return False

    # AUTO / status / scan
    def _cmd_auto(self, arg: str) -> bool:
        if arg in ("on", "1", "true"):
            new = set_auto(True)
        elif arg in ("off", "0", "false"):
            new = set_auto(False)
        else:
            new = set_auto(not get_auto())
        print(f"🔁 AUTO = {'ON' if new else 'OFF'}")
        return True

    def _cmd_status(self) -> bool:
        print(f"AUTO  = {'ON' if get_auto() else 'OFF'}")
        print(f"ESP32 = {ESP32_URL}")
        print(f"目前姿勢 = {self.pose}")
        return True

    def _cmd_scan(self) -> bool:
        vision = get_vision()
        if vision is None:
            print("❌ 取不到 /vision")
            return True
        print(f"👁 {vision}")
        if not (vision.get("target_found") and vision.get("dest_found")):
            print("⚠ 沒同時找到 target+dest")
            return True
        seq = compute_sequence(vision["tx"], vision["ty"],
                               vision["dx"], vision["dy"])
        if seq is None:
            print("⚠ 超出可達範圍")
            return True
        print(f"📤 {json.dumps(seq)}")
        post_execute(seq)
        return True

    # 手動推姿勢
    def _send_pick(self) -> None:
        post_execute({"pick": dict(self.pose)})

    def _cmd_home(self) -> bool:
        post_execute({"home": HOME_ANGLES})
        self.pose = dict(INITIAL_POSE)
        return True

    def _cmd_px(self, arg: str) -> bool:
        try:
            u, v = (float(s) for s in arg.split(","))
        except ValueError:
            print("❌ 格式: px=u,v")
            return True
        x, y = pixel_to_real(u, v)
        print(f"🤖 ({u},{v}) → x={x:.1f}mm y={y:.1f}mm")
        result = xyz_to_angles(x, y, OBJECT_Z_MM)
        if result is None:
            print("⚠ 超出可達範圍")
            return True
        self.pose.update(result)
        self._send_pick()
        return True

    def _cmd_xyz(self, cmd: str) -> bool:
        try:
            x, y, z = (float(s) for s in cmd.split(","))
        except ValueError:
            print("❌ 格式: x,y,z")
            return True
        result = xyz_to_angles(x, y, z)
        if result is None:
            print("⚠ 超出可達範圍")
            return True
        self.pose.update(result)
        print(f"🤖 x={x} y={y} z={z} → {result}")
        self._send_pick()
        return True

    def _cmd_single(self, ch: str, arg: str) -> bool:
        try:
            val = clamp(ch, int(arg))
        except ValueError:
            print("❌ 格式: chX=整數")
            return True
        self.pose[ch] = val
        self._send_pick()
        print(f"{ch}={val}")
        return True

    def _cmd_all(self, arg: str) -> bool:
        parts = arg.split(",")
        if len(parts) != 4:
            print("❌ 格式: all=ch0,ch1,ch2,ch3")
            return True
        try:
            pose = {ch: clamp(ch, int(v)) for ch, v in zip(self.SINGLE_CH, parts)}
        except ValueError:
            print("❌ 格式錯誤")
            return True
        self.pose = pose
        post_execute({"pick": pose})
        print(f"全軸設定: {pose}")
        return True


# ── 入口 ─────────────────────────────────────────
def _start_flask() -> None:
    app.run(host=HOST, port=PORT, debug=False, use_reloader=False, threaded=True)


def main() -> None:
    log.info("🚀 Flask server 啟動在 %s:%d", HOST, PORT)
    threading.Thread(target=_start_flask, daemon=True).start()
    threading.Thread(target=vision_poller, daemon=True).start()
    Terminal().run()


if __name__ == "__main__":
    main()
