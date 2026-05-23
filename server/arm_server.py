from flask import Flask, request, jsonify
import math
import numpy as np
import cv2
import json
import threading
import requests

app = Flask(__name__)

ARM = {
    "home": {"ch0": 90, "ch1": 90, "ch2": 120, "ch3": 5},
}

COORD = {
    "object_z_mm": 10,
    "place_ch0_offset": -5,  # 放置時 ch0 補償
}

ESP32_URL = "http://10.20.171.112:80"

# ── Homography ────────────────────────────────────
pixel_pts = np.array([
    [142, 151],
    [92,  138],
    [217, 119],
    [96,  202],
], dtype=np.float32)

real_pts = np.array([
    [145,    0],
    [158,   50],
    [180,  -75],
    [95,    48],
], dtype=np.float32)

H, _ = cv2.findHomography(pixel_pts, real_pts)

def pixel_to_real_h(u, v):
    pt = np.array([[[u, v]]], dtype=np.float32)
    real = cv2.perspectiveTransform(pt, H)
    return float(real[0][0][0]), float(real[0][0][1])

# ── 手臂參數 ──────────────────────────────────────
A1 = 81.4
A2 = 84.0
A3 = 67.0

CL_TABLE  = [13.6, 36.1, 49.2, 60.4, 71.1]
CH1_TABLE = [175,  160,  153,  140,  135]
CR_TABLE  = [2.9,  23.1, 33.7, 41.6, 47.6]
CH2_TABLE = [60,   40,   25,   10,   0]

# ── IK ───────────────────────────────────────────
def cal_arm_angle(x, y, z):
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
    isreachable = not (
        clamta < math.radians(-97) or
        clamta > math.radians(137) or
        cL < math.radians(-15) or
        cL > math.radians(128) or
        beta < math.radians(25) or
        beta > math.radians(160) or
        cR < math.radians(-19.5) or
        cR > math.radians(116)
    )
    return isreachable, clamta, cL, cR

def angles_to_servo(clamta, cL, cR):
    ch0 = 90 + math.degrees(clamta)
    if clamta < 0:
        ch0 = ch0 - 4
    elif clamta > 0:
        ch0 = ch0 - 5

    ch1 = round(np.interp(math.degrees(cL), CL_TABLE, CH1_TABLE))
    ch1 = ch1 - 3

    ch2 = round(np.interp(math.degrees(cR), CR_TABLE, CH2_TABLE))

    ch0 = max(5,  min(175, int(ch0)))
    ch1 = max(90, min(175, int(ch1)))
    ch2 = max(0,  min(120, int(ch2)))
    return ch0, ch1, ch2

def xyz_to_angles(x_mm, y_mm, z_mm):
    isreachable, clamta, cL, cR = cal_arm_angle(x_mm, y_mm, z_mm)
    if not isreachable:
        return None
    ch0, ch1, ch2 = angles_to_servo(clamta, cL, cR)
    return {"ch0": ch0, "ch1": ch1, "ch2": ch2}

def build(ik_result, ch3, ch0_offset=0):
    return {
        "ch0": max(5, min(175, ik_result["ch0"] + ch0_offset)),
        "ch1": ik_result["ch1"],
        "ch2": ik_result["ch2"],
        "ch3": ch3,
    }

# ── 發送到 ESP32 ──────────────────────────────────
def send(angles):
    try:
        resp = requests.post(f"{ESP32_URL}/execute", json=angles, timeout=30)
        if resp.status_code == 200:
            print("✅ ESP32 執行完成")
        else:
            print(f"❌ 失敗 code={resp.status_code}")
    except Exception as e:
        print(f"❌ 連線錯誤: {e}")

# ── Flask 路由 ────────────────────────────────────
@app.route('/arm', methods=['POST'])
def arm_control():
    data = request.get_json()
    print(f"📍 收到像素座標: {data}")

    tx, ty = pixel_to_real_h(data['tx'], data['ty'])
    dx, dy = pixel_to_real_h(data['dx'], data['dy'])
    z_mm   = COORD['object_z_mm']
    offset = COORD['place_ch0_offset']

    print(f"📐 目標: x={tx:.1f} y={ty:.1f} z={z_mm}mm")
    print(f"📐 目的地: x={dx:.1f} y={dy:.1f} z={z_mm}mm")

    target = xyz_to_angles(tx, ty, z_mm)
    dest   = xyz_to_angles(dx, dy, z_mm)

    if not target or not dest:
        print("⚠️ 超出可達範圍")
        return jsonify({"error": "out of range"}), 400

    angles = {
        "pick":    build(target, 5),
        "grip":    build(target, 90),
        "place":   build(dest,   90, ch0_offset=offset),
        "release": build(dest,   5,  ch0_offset=offset),
        "home":    ARM["home"],
    }

    print(f"📤 回傳角度:\n{json.dumps(angles, indent=2)}")
    return jsonify(angles)

@app.route('/execute', methods=['POST'])
def execute():
    data = request.get_json()
    print(f"📥 直接角度: {json.dumps(data, indent=2)}")
    return jsonify(data)

@app.route('/health', methods=['GET'])
def health():
    return "OK", 200

# ── 終端控制 ──────────────────────────────────────
def terminal():
    print("\n🚀 手臂控制工具")
    print(f"ESP32: {ESP32_URL}")
    print("指令:")
    print("  x,y,z          → 真實座標(mm)")
    print("  px=u,v         → 像素座標換算")
    print("  ch0/1/2/3=角度 → 直接控制")
    print("  all=ch0,ch1,ch2,ch3")
    print("  home / q\n")

    current = {"ch0": 90, "ch1": 153, "ch2": 25, "ch3": 5}

    while True:
        try:
            cmd = input("指令> ").strip().lower()
        except EOFError:
            break

        if cmd == 'q':
            break

        elif cmd == 'home':
            send({"home": ARM["home"]})
            current = {"ch0": 90, "ch1": 153, "ch2": 25, "ch3": 5}

        elif cmd.startswith('px='):
            try:
                parts = cmd[3:].split(',')
                u, v = float(parts[0]), float(parts[1])
                x, y = pixel_to_real_h(u, v)
                print(f"🤖 ({u},{v}) → x={x:.1f}mm y={y:.1f}mm")
                result = xyz_to_angles(x, y, COORD['object_z_mm'])
                if result:
                    current.update(result)
                    send({"pick": {**current}})
            except Exception as e:
                print(f"❌ 錯誤: {e}")

        elif cmd.count(',') == 2:
            try:
                parts = cmd.split(',')
                x_mm = float(parts[0])
                y_mm = float(parts[1])
                z_mm = float(parts[2])
                result = xyz_to_angles(x_mm, y_mm, z_mm)
                if result:
                    current.update(result)
                    print(f"🤖 x={x_mm} y={y_mm} z={z_mm} → {result}")
                    send({"pick": {**current}})
            except Exception as e:
                print(f"❌ 錯誤: {e}")

        elif cmd.startswith('ch0='):
            try:
                val = max(5, min(175, int(cmd.split('=')[1])))
                current["ch0"] = val
                send({"pick": {**current}})
                print(f"ch0={val}")
            except:
                print("格式錯誤")

        elif cmd.startswith('ch1='):
            try:
                val = max(90, min(175, int(cmd.split('=')[1])))
                current["ch1"] = val
                send({"pick": {**current}})
                print(f"ch1={val}")
            except:
                print("格式錯誤")

        elif cmd.startswith('ch2='):
            try:
                val = max(0, min(120, int(cmd.split('=')[1])))
                current["ch2"] = val
                send({"pick": {**current}})
                print(f"ch2={val}")
            except:
                print("格式錯誤")

        elif cmd.startswith('ch3='):
            try:
                val = max(5, min(90, int(cmd.split('=')[1])))
                current["ch3"] = val
                send({"pick": {**current}})
                print(f"ch3={val}")
            except:
                print("格式錯誤")

        elif cmd.startswith('all='):
            try:
                parts = cmd.split('=')[1].split(',')
                ch0 = max(5,  min(175, int(parts[0])))
                ch1 = max(90, min(175, int(parts[1])))
                ch2 = max(0,  min(120, int(parts[2])))
                ch3 = max(5,  min(90,  int(parts[3])))
                current = {"ch0": ch0, "ch1": ch1, "ch2": ch2, "ch3": ch3}
                send({"pick": current})
                print(f"全軸設定: {current}")
            except:
                print("格式錯誤")

        else:
            print("不認識的指令")

    send({"home": ARM["home"]})
    print("✅ 結束")

if __name__ == '__main__':
    print("🚀 Flask server 啟動在 0.0.0.0:5000")
    t = threading.Thread(target=lambda: app.run(
        host='0.0.0.0', port=5000, debug=False, use_reloader=False))
    t.daemon = True
    t.start()
    terminal()
