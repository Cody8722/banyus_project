"""Banyus arm — Flask backend.

接收 ESP32 送來的像素座標，做 IK 計算後回傳 5 階段伺服角度。
也提供 /execute 與 /health，以及終端互動工具。

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

# ── 設定常數 ─────────────────────────────────────
HOST = "0.0.0.0"
PORT = 5000
ESP32_URL     = "http://banyus.local"   # ESP32 端註冊的 mDNS 名稱，免擔心 DHCP 變動
ESP32_TIMEOUT = 30   # s

OBJECT_Z_MM      = 10
PLACE_CH0_OFFSET = -5   # 放置時的 ch0 機構偏移補償

HOME_ANGLES   = {"ch0": 90, "ch1": 90,  "ch2": 120, "ch3": 5}
INITIAL_POSE  = {"ch0": 90, "ch1": 153, "ch2": 25,  "ch3": 5}

CH3_OPEN   = 5
CH3_CLOSED = 90

# 各軸允許範圍（與 ESP32 端一致）
SERVO_LIMITS = {
    "ch0": (5, 175),
    "ch1": (90, 175),
    "ch2": (0, 120),
    "ch3": (5, 90),
}

# 由實機校正得到的機構補償（不影響 IK 數學）
CH0_OFFSET_NEG = -4   # clamta < 0 時
CH0_OFFSET_POS = -5   # clamta > 0 時
CH1_OFFSET     = -3

# ── Logger ───────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("arm")
logging.getLogger("werkzeug").setLevel(logging.WARNING)

# ── Homography（像素 → 真實 mm） ───────────────────
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


# ── IK 參數（保留原值） ──────────────────────────
A1 = 81.4
A2 = 84.0
A3 = 67.0

# cL (deg) → ch1 線性內插
CL_TABLE  = [13.6, 36.1, 49.2, 60.4, 71.1]
CH1_TABLE = [175,  160,  153,  140,  135]

# cR (deg) → ch2 線性內插
CR_TABLE  = [2.9,  23.1, 33.7, 41.6, 47.6]
CH2_TABLE = [60,   40,   25,   10,   0]

# 可達範圍（rad），與原本完全相同
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


# ── 與 ESP32 溝通（共用 Session） ─────────────────
_session = requests.Session()


def send_to_esp32(payload: dict) -> bool:
    try:
        resp = _session.post(f"{ESP32_URL}/execute", json=payload, timeout=ESP32_TIMEOUT)
        if resp.status_code == 200:
            log.info("✅ ESP32 已接受（背景執行中）")
            return True
        if resp.status_code == 503:
            log.warning("⚠ ESP32 忙碌中，請稍候")
        else:
            log.error("❌ ESP32 失敗 code=%d body=%s", resp.status_code, resp.text)
    except requests.RequestException as e:
        log.error("❌ ESP32 連線錯誤: %s", e)
    return False


# ── Flask routes ─────────────────────────────────
app = Flask(__name__)


@app.route("/arm", methods=["POST"])
def arm_control():
    data = request.get_json(silent=True) or {}
    try:
        tx_px = float(data["tx"])
        ty_px = float(data["ty"])
        dx_px = float(data["dx"])
        dy_px = float(data["dy"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"error": "missing or invalid tx/ty/dx/dy"}), 400

    log.info("📍 像素 target=(%.0f,%.0f) dest=(%.0f,%.0f)",
             tx_px, ty_px, dx_px, dy_px)

    tx, ty = pixel_to_real(tx_px, ty_px)
    dx, dy = pixel_to_real(dx_px, dy_px)
    log.info("📐 目標   x=%.1f y=%.1f z=%d", tx, ty, OBJECT_Z_MM)
    log.info("📐 目的地 x=%.1f y=%.1f z=%d", dx, dy, OBJECT_Z_MM)

    target = xyz_to_angles(tx, ty, OBJECT_Z_MM)
    dest   = xyz_to_angles(dx, dy, OBJECT_Z_MM)
    if target is None or dest is None:
        log.warning("⚠ 超出可達範圍")
        return jsonify({"error": "out of range"}), 400

    angles = {
        "pick":    build_phase(target, CH3_OPEN),
        "grip":    build_phase(target, CH3_CLOSED),
        "place":   build_phase(dest,   CH3_CLOSED, ch0_offset=PLACE_CH0_OFFSET),
        "release": build_phase(dest,   CH3_OPEN,   ch0_offset=PLACE_CH0_OFFSET),
        "home":    HOME_ANGLES,
    }
    log.info("📤 回傳角度:\n%s", json.dumps(angles, indent=2))
    return jsonify(angles)


@app.route("/execute", methods=["POST"])
def execute():
    data = request.get_json(silent=True) or {}
    log.info("📥 直接角度:\n%s", json.dumps(data, indent=2))
    return jsonify(data)


@app.route("/health", methods=["GET"])
def health():
    return "OK", 200


# ── 終端互動 ─────────────────────────────────────
class Terminal:
    HELP = (
        "\n🚀 手臂控制工具\n"
        "ESP32: {url}\n"
        "指令:\n"
        "  x,y,z          → 真實座標(mm)\n"
        "  px=u,v         → 像素座標換算\n"
        "  ch0/1/2/3=角度 → 直接控制\n"
        "  all=ch0,ch1,ch2,ch3\n"
        "  home / q\n"
    )

    SINGLE_CH = ("ch0", "ch1", "ch2", "ch3")

    def __init__(self) -> None:
        self.pose: dict = dict(INITIAL_POSE)

    # 主迴圈
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
        send_to_esp32({"home": HOME_ANGLES})
        print("✅ 結束")

    # 命令分派
    def _dispatch(self, cmd: str) -> bool:
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

    def _send_pick(self) -> None:
        send_to_esp32({"pick": dict(self.pose)})

    # 各指令處理
    def _cmd_home(self) -> bool:
        send_to_esp32({"home": HOME_ANGLES})
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
        send_to_esp32({"pick": pose})
        print(f"全軸設定: {pose}")
        return True


# ── 入口 ─────────────────────────────────────────
def _start_flask() -> None:
    app.run(host=HOST, port=PORT, debug=False, use_reloader=False, threaded=True)


def main() -> None:
    log.info("🚀 Flask server 啟動在 %s:%d", HOST, PORT)
    t = threading.Thread(target=_start_flask, daemon=True)
    t.start()
    Terminal().run()


if __name__ == "__main__":
    main()
