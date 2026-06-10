#!/usr/bin/env python3
"""
BlueOS Serial Reader Extension — 带 IMU 航位推算
"""

import json
import math
import time
import threading
from collections import deque
from flask import Flask, request as flask_request
from serial_driver import SerialDriver
from blueoshelper import request as blueos_request

app = Flask(__name__, static_url_path="/static", static_folder="static")

# ── 常量 ────────────────────────────────────────────────────────────────────
MAVLINK_BASE   = "http://127.0.0.1:6040/v1/mavlink/vehicles/1/components/1/messages"
MAX_TRACK_PTS  = 8000
DR_HZ          = 10          # 航位推算频率 Hz
ACCEL_DEADBAND = 0.05        # m/s²，低于此值视为静止噪声归零
STILL_THRESH   = 0.08        # m/s²，连续静止判定阈值（归零速度用）
STILL_COUNT    = 30          # 连续 N 帧静止则速度归零
G              = 9.80665     # 重力加速度

# ── 共享状态 ─────────────────────────────────────────────────────────────────
_telem: dict        = {}
_telem_lock         = threading.Lock()

_track: deque       = deque(maxlen=MAX_TRACK_PTS)
_track_lock         = threading.Lock()

# 航位推算状态（后台线程独占写，REST 只读快照）
_dr_state: dict     = {
    "x": 0.0, "y": 0.0, "z": 0.0,          # 位置 m（局部坐标系）
    "vx": 0.0, "vy": 0.0, "vz": 0.0,        # 速度 m/s（世界系）
    "dist": 0.0,                              # 累计路程 m
    "drift_warn": False,                      # 漂移警告
    "running_time": 0.0,                      # 积分时长 s
    "still_frames": 0,
}
_dr_lock            = threading.Lock()
_dr_reset_flag      = threading.Event()       # 外部触发重置

# ── MAVLink 拉取 ─────────────────────────────────────────────────────────────

def _fetch(msg: str):
    try:
        raw = blueos_request(f"{MAVLINK_BASE}/{msg}")
        if raw is None:
            return None
        return json.loads(raw).get("message", {})
    except Exception:
        return None

# ── 旋转矩阵：机体 → 世界（NED） ────────────────────────────────────────────

def _body_to_world(ax_b, ay_b, az_b, roll, pitch, yaw):
    """
    把机体系加速度旋转到世界系（NED），同时减去重力。
    roll/pitch/yaw 单位：弧度。
    返回 (ax_w, ay_w, az_w) 单位 m/s²。
    """
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)

    # ZYX 顺序旋转矩阵 R = Rz(yaw)*Ry(pitch)*Rx(roll)
    r00 = cy*cp;  r01 = cy*sp*sr - sy*cr;  r02 = cy*sp*cr + sy*sr
    r10 = sy*cp;  r11 = sy*sp*sr + cy*cr;  r12 = sy*sp*cr - cy*sr
    r20 = -sp;    r21 = cp*sr;             r22 = cp*cr

    # 世界系加速度 = R * a_body，然后减去重力（NED 中 g 在 z 正方向）
    ax_w =  r00*ax_b + r01*ay_b + r02*az_b
    ay_w =  r10*ax_b + r11*ay_b + r12*az_b
    az_w = (r20*ax_b + r21*ay_b + r22*az_b) - G

    return ax_w, ay_w, az_w

# ── 航位推算主循环 ────────────────────────────────────────────────────────────

def _dr_loop():
    dt       = 1.0 / DR_HZ
    last_ts  = None          # 上一条 RAW_IMU 的 time_usec，用于对齐实际 dt

    while True:
        # ── 外部重置 ──────────────────────────────────────────────────────
        if _dr_reset_flag.is_set():
            _dr_reset_flag.clear()
            with _dr_lock:
                _dr_state.update(x=0, y=0, z=0, vx=0, vy=0, vz=0,
                                 dist=0, drift_warn=False,
                                 running_time=0, still_frames=0)
            with _track_lock:
                _track.clear()
            last_ts = None

        t0 = time.time()

        # ── 拉取 IMU + 姿态 ───────────────────────────────────────────────
        imu      = _fetch("SCALED_IMU2") or _fetch("RAW_IMU")
        attitude = _fetch("ATTITUDE")
        ahrs2    = _fetch("AHRS2")
        ekf      = _fetch("EKF_STATUS_REPORT")

        # ── 更新遥测快照 ──────────────────────────────────────────────────
        telem_update = {"ts": time.time()}
        if attitude:
            telem_update["roll"]  = round(attitude.get("roll",  0), 5)
            telem_update["pitch"] = round(attitude.get("pitch", 0), 5)
            telem_update["yaw"]   = round(attitude.get("yaw",   0), 5)
            telem_update["rollspeed"]  = round(attitude.get("rollspeed",  0), 4)
            telem_update["pitchspeed"] = round(attitude.get("pitchspeed", 0), 4)
            telem_update["yawspeed"]   = round(attitude.get("yawspeed",   0), 4)
        if ahrs2:
            telem_update["altitude"] = round(ahrs2.get("altitude", 0), 3)
        if ekf:
            telem_update["ekf_vel_variance"]  = round(ekf.get("velocity_variance",  0), 4)
            telem_update["ekf_pos_horiz"]     = round(ekf.get("pos_horiz_variance", 0), 4)
            telem_update["ekf_pos_vert"]      = round(ekf.get("pos_vert_variance",  0), 4)
        if imu:
            # SCALED_IMU2: xacc/yacc/zacc 单位 mG，换算 m/s²
            # RAW_IMU:      同上（ArduSub 统一用 mG）
            scale = 9.80665 / 1000.0
            telem_update["ax_raw"] = round(imu.get("xacc", 0) * scale, 4)
            telem_update["ay_raw"] = round(imu.get("yacc", 0) * scale, 4)
            telem_update["az_raw"] = round(imu.get("zacc", 0) * scale, 4)
            telem_update["gx_raw"] = round(imu.get("xgyro", 0) * 0.001, 5)  # mrad/s → rad/s
            telem_update["gy_raw"] = round(imu.get("ygyro", 0) * 0.001, 5)
            telem_update["gz_raw"] = round(imu.get("zgyro", 0) * 0.001, 5)

        with _telem_lock:
            _telem.update(telem_update)

        # ── 航位推算积分 ──────────────────────────────────────────────────
        if imu and attitude:
            scale = 9.80665 / 1000.0
            ax_b = imu.get("xacc", 0) * scale
            ay_b = imu.get("yacc", 0) * scale
            az_b = imu.get("zacc", 0) * scale

            roll  = attitude.get("roll",  0)
            pitch = attitude.get("pitch", 0)
            yaw   = attitude.get("yaw",   0)

            ax_w, ay_w, az_w = _body_to_world(ax_b, ay_b, az_b, roll, pitch, yaw)

            # 去除极小噪声
            def deadband(v): return v if abs(v) > ACCEL_DEADBAND else 0.0
            ax_w, ay_w, az_w = deadband(ax_w), deadband(ay_w), deadband(az_w)

            with _dr_lock:
                sf = _dr_state["still_frames"]
                acc_mag = math.sqrt(ax_w**2 + ay_w**2 + az_w**2)

                # 静止检测：若加速度持续很小，将速度归零（抑制漂移）
                if acc_mag < STILL_THRESH:
                    sf += 1
                else:
                    sf = 0

                if sf >= STILL_COUNT:
                    _dr_state["vx"] = 0.0
                    _dr_state["vy"] = 0.0
                    _dr_state["vz"] = 0.0

                _dr_state["still_frames"] = sf

                # 速度积分
                _dr_state["vx"] += ax_w * dt
                _dr_state["vy"] += ay_w * dt
                _dr_state["vz"] += az_w * dt

                # 位置积分
                dx = _dr_state["vx"] * dt
                dy = _dr_state["vy"] * dt
                dz = _dr_state["vz"] * dt
                _dr_state["x"] += dx
                _dr_state["y"] += dy
                _dr_state["z"] += dz
                _dr_state["dist"] += math.sqrt(dx**2 + dy**2 + dz**2)
                _dr_state["running_time"] += dt

                # 漂移警告：积分超过 60 秒后提示
                _dr_state["drift_warn"] = _dr_state["running_time"] > 60

                x, y, z = _dr_state["x"], _dr_state["y"], _dr_state["z"]
                dist     = _dr_state["dist"]
                dw       = _dr_state["drift_warn"]
                spd      = math.sqrt(_dr_state["vx"]**2 + _dr_state["vy"]**2 + _dr_state["vz"]**2)

            # 写入遥测快照（供前端实时读）
            with _telem_lock:
                _telem.update({
                    "dr_x": round(x, 3), "dr_y": round(y, 3), "dr_z": round(z, 3),
                    "dr_dist": round(dist, 2),
                    "dr_speed": round(spd, 3),
                    "dr_time":  round(_dr_state["running_time"], 1),
                    "drift_warn": dw,
                    "ax_w": round(ax_w, 4), "ay_w": round(ay_w, 4), "az_w": round(az_w, 4),
                })

            # 记录轨迹点（每次积分都记，降采样到 DR_HZ/2）
            if int(_dr_state["running_time"] * DR_HZ) % 2 == 0:
                depth = ahrs2.get("altitude", 0) if ahrs2 else 0.0
                point = {
                    "ts":    time.time(),
                    "x":     round(x, 3),
                    "y":     round(y, 3),
                    "z":     round(z, 3),
                    "depth": round(depth, 3),
                    "roll":  round(roll, 4),
                    "pitch": round(pitch, 4),
                    "yaw":   round(yaw, 4),
                    "speed": round(spd, 3),
                }
                with _track_lock:
                    _track.append(point)

        # ── 保持频率 ──────────────────────────────────────────────────────
        elapsed = time.time() - t0
        sleep_t = max(0.0, dt - elapsed)
        time.sleep(sleep_t)


_dr_thread = threading.Thread(target=_dr_loop, daemon=True)
_dr_thread.start()

# ── API 类 ───────────────────────────────────────────────────────────────────

class API:
    def __init__(self, driver: SerialDriver):
        self.driver = driver

    def get_status(self):        return self.driver.get_status()
    def get_history_since(self, since, limit=2000): return self.driver.get_history_since(since, limit)
    def export_history(self, limit=30000):          return self.driver.export_history(limit)
    def set_enabled(self, enabled):
        if enabled in ["true", "false"]:
            return self.driver.set_enabled(enabled == "true")
        return False
    def set_port(self, port):    return self.driver.set_port(port)
    def set_baud(self, baud):
        try:    return self.driver.set_baud(int(baud))
        except: return False
    def clear_history(self):
        self.driver.clear_history(); return True
    def list_ports(self):        return self.driver.list_ports()


if __name__ == "__main__":
    driver = SerialDriver()
    api    = API(driver)

    # ── 串口路由 ──────────────────────────────────────────────────────────────
    @app.route("/register_service")
    def register_service(): return app.send_static_file("service.json")

    @app.route("/")
    def root(): return app.send_static_file("index.html")

    @app.route("/get_status")
    def get_status(): return json.dumps(api.get_status())

    @app.route("/get_history_since/<int:since>")
    def get_history_since(since): return json.dumps(api.get_history_since(since))

    @app.route("/export_history")
    def export_history():
        try:    limit = int(flask_request.args.get("limit", 30000))
        except: limit = 30000
        return json.dumps(api.export_history(limit))

    @app.route("/enable/<enable>")
    def set_enabled(enable): return str(api.set_enabled(enable))

    @app.route("/set_port/<path:port>")
    def set_port(port): return str(api.set_port(port))

    @app.route("/set_baud/<baud>")
    def set_baud(baud): return str(api.set_baud(baud))

    @app.route("/clear_history")
    def clear_history(): return str(api.clear_history())

    @app.route("/list_ports")
    def list_ports(): return json.dumps(api.list_ports())

    # ── 遥测 & 航位推算路由 ───────────────────────────────────────────────────
    @app.route("/get_telemetry")
    def get_telemetry():
        with _telem_lock:
            return json.dumps(_telem)

    @app.route("/get_track")
    def get_track():
        with _track_lock:
            return json.dumps(list(_track))

    @app.route("/get_track_since/<since_ts>")
    def get_track_since(since_ts):
        try:    ts = float(since_ts)
        except: ts = 0.0
        with _track_lock:
            return json.dumps([p for p in _track if p["ts"] > ts])

    @app.route("/get_dr_state")
    def get_dr_state():
        with _dr_lock:
            return json.dumps(dict(_dr_state))

    @app.route("/reset_dr")
    def reset_dr():
        """重置航位推算（清零位置、速度、轨迹）"""
        _dr_reset_flag.set()
        return json.dumps(True)

    driver.start()
    app.run(host="0.0.0.0", port=9001)
