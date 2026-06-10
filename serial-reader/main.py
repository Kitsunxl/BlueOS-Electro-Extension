#!/usr/bin/env python3
"""
BlueOS Serial Reader Extension — 带 IMU 航位推算
"""

import json
import math
import os
import time
import threading
from collections import deque
from flask import Flask, request as flask_request
from serial_driver import SerialDriver
from blueoshelper import request as blueos_request

app = Flask(__name__, static_url_path="/static", static_folder="static")

# ── 常量 ────────────────────────────────────────────────────────────────────
MAVLINK_PATH   = "/v1/mavlink/vehicles/1/components/1/messages"
MAVLINK_HOSTS  = [
    os.environ.get("MAVLINK2REST_HOST", "host.docker.internal:6040"),
    "127.0.0.1:6040",
]
MAVLINK_BASES  = [f"http://{host}{MAVLINK_PATH}" for host in dict.fromkeys(MAVLINK_HOSTS) if host]
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

_mav_status: dict   = {}
_mav_status_lock    = threading.Lock()
_active_mavlink_base = MAVLINK_BASES[0] if MAVLINK_BASES else ""

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
_gravity_bias       = None                    # 世界系静止加速度基线
_gravity_samples    = []
GRAVITY_CAL_SAMPLES = 30

# ── MAVLink 拉取 ─────────────────────────────────────────────────────────────

def _fetch(msg: str):
    global _active_mavlink_base
    last_error = "no MAVLink2REST base URL configured"
    last_base = ""
    for base in MAVLINK_BASES:
        last_base = base
        try:
            raw = blueos_request(f"{base}/{msg}")
            if raw is None:
                last_error = "empty response or request failed"
                continue
            message = json.loads(raw).get("message", {})
            if not message:
                last_error = "message field missing or empty"
                continue
            _active_mavlink_base = base
            with _mav_status_lock:
                _mav_status[msg] = {
                    "ok": True,
                    "base": base,
                    "last_attempt": round(time.time(), 3),
                    "error": None,
                }
            return message
        except Exception as error:
            last_error = str(error)

    with _mav_status_lock:
        _mav_status[msg] = {
            "ok": False,
            "base": last_base,
            "last_attempt": round(time.time(), 3),
            "error": last_error,
        }
    return None


def _fetch_imu():
    for msg in ("SCALED_IMU2", "SCALED_IMU", "RAW_IMU"):
        imu = _fetch(msg)
        if imu:
            return msg, imu
    return None, None


def _imu_accel_scale(_msg: str):
    # SCALED_IMU/RAW_IMU acceleration fields are in milli-g.
    return G / 1000.0


def _imu_gyro_scale(_msg: str):
    # SCALED_IMU/RAW_IMU gyro fields are in millirad/s.
    return 0.001


# ── 旋转矩阵：机体 → 世界（NED） ────────────────────────────────────────────

def _body_to_world(ax_b, ay_b, az_b, roll, pitch, yaw):
    """
    把机体系加速度旋转到世界系。
    roll/pitch/yaw 单位：弧度。
    返回世界系的含重力加速度/比力 (ax_w, ay_w, az_w)，单位 m/s²。
    """
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)

    # ZYX 顺序旋转矩阵 R = Rz(yaw)*Ry(pitch)*Rx(roll)
    r00 = cy*cp;  r01 = cy*sp*sr - sy*cr;  r02 = cy*sp*cr + sy*sr
    r10 = sy*cp;  r11 = sy*sp*sr + cy*cr;  r12 = sy*sp*cr - cy*sr
    r20 = -sp;    r21 = cp*sr;             r22 = cp*cr

    # 世界系加速度 = R * a_body。这里先不去重力，后续用静止基线扣除。
    ax_w =  r00*ax_b + r01*ay_b + r02*az_b
    ay_w =  r10*ax_b + r11*ay_b + r12*az_b
    az_w =  r20*ax_b + r21*ay_b + r22*az_b

    return ax_w, ay_w, az_w

# ── 航位推算主循环 ────────────────────────────────────────────────────────────

def _dr_loop():
    global _gravity_bias, _gravity_samples
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
            _gravity_bias = None
            _gravity_samples = []
            last_ts = None

        t0 = time.time()

        # ── 拉取 IMU + 姿态 ───────────────────────────────────────────────
        imu_msg, imu = _fetch_imu()
        attitude = _fetch("ATTITUDE")
        ahrs2    = _fetch("AHRS2")
        ekf      = _fetch("EKF_STATUS_REPORT")

        # ── 更新遥测快照 ──────────────────────────────────────────────────
        telem_update = {"ts": time.time()}
        with _mav_status_lock:
            status_snapshot = dict(_mav_status)
        telem_update["mavlink_base"] = _active_mavlink_base
        telem_update["mavlink_status"] = status_snapshot
        telem_update["imu_message"] = imu_msg if imu else None
        telem_update["imu_ok"] = bool(imu)
        telem_update["attitude_ok"] = bool(attitude)
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
            scale = _imu_accel_scale(imu_msg)
            telem_update["ax_raw"] = round(imu.get("xacc", 0) * scale, 4)
            telem_update["ay_raw"] = round(imu.get("yacc", 0) * scale, 4)
            telem_update["az_raw"] = round(imu.get("zacc", 0) * scale, 4)
            gyro_scale = _imu_gyro_scale(imu_msg)
            telem_update["gx_raw"] = round(imu.get("xgyro", 0) * gyro_scale, 5)
            telem_update["gy_raw"] = round(imu.get("ygyro", 0) * gyro_scale, 5)
            telem_update["gz_raw"] = round(imu.get("zgyro", 0) * gyro_scale, 5)

        with _telem_lock:
            _telem.update(telem_update)

        # ── 航位推算积分 ──────────────────────────────────────────────────
        if imu and attitude:
            scale = _imu_accel_scale(imu_msg)
            ax_b = imu.get("xacc", 0) * scale
            ay_b = imu.get("yacc", 0) * scale
            az_b = imu.get("zacc", 0) * scale

            roll  = attitude.get("roll",  0)
            pitch = attitude.get("pitch", 0)
            yaw   = attitude.get("yaw",   0)

            ax_g, ay_g, az_g = _body_to_world(ax_b, ay_b, az_b, roll, pitch, yaw)

            if _gravity_bias is None:
                _gravity_samples.append((ax_g, ay_g, az_g))
                if len(_gravity_samples) >= GRAVITY_CAL_SAMPLES:
                    n = len(_gravity_samples)
                    _gravity_bias = (
                        sum(p[0] for p in _gravity_samples) / n,
                        sum(p[1] for p in _gravity_samples) / n,
                        sum(p[2] for p in _gravity_samples) / n,
                    )
                ax_w, ay_w, az_w = 0.0, 0.0, 0.0
            else:
                ax_w = ax_g - _gravity_bias[0]
                ay_w = ay_g - _gravity_bias[1]
                az_w = az_g - _gravity_bias[2]

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
                    "ax_g": round(ax_g, 4), "ay_g": round(ay_g, 4), "az_g": round(az_g, 4),
                    "gravity_calibrated": _gravity_bias is not None,
                    "gravity_cal_samples": len(_gravity_samples),
                    "gravity_bias": [round(v, 4) for v in _gravity_bias] if _gravity_bias else None,
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

    @app.route("/health")
    def health():
        return json.dumps({"ok": True, "service": "serial-reader"})

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
    app.run(host="0.0.0.0", port=9001, threaded=True)
