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
ACCEL_DEADBAND = 0.12        # m/s²，低于此值视为静止噪声归零
STILL_THRESH   = 0.18        # m/s²，连续静止判定阈值（归零速度用）
STILL_COUNT    = 30          # 连续 N 帧静止则速度归零
ACCEL_FILTER_ALPHA = 0.25     # 世界系线性加速度低通系数，越小越平滑
ACCEL_JUMP_LIMIT   = 0.8      # m/s²，单帧最大允许跳变
G              = 9.80665     # 重力加速度
IMU_BODY_AXIS_MAP = os.environ.get("IMU_BODY_AXIS_MAP", "x,z,y")
DT_MIN, DT_MAX     = 0.01, 0.5     # 真实 dt 钳位（s），抑制拉取抖动/卡顿造成的积分失真
STILL_BIAS_ALPHA   = 0.01          # 静止窗口内重力/零偏基线的在线重估系数

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
_accel_scale        = None                    # raw accel count -> m/s²
_accel_scale_samples = []
_filtered_accel     = None                    # 世界系线性加速度滤波状态
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


def _fetch_imu(pref: str = None):
    """优先尝试已锁定的 IMU 源 pref，避免每圈都从头试 3 种消息。"""
    order = []
    if pref:
        order.append(pref)
    for msg in ("SCALED_IMU2", "SCALED_IMU", "RAW_IMU"):
        if msg not in order:
            order.append(msg)
    for msg in order:
        imu = _fetch(msg)
        if imu:
            return msg, imu
    return None, None


def _imu_accel_scale(_msg: str):
    # SCALED_IMU/RAW_IMU acceleration fields are in milli-g.
    return _accel_scale if _accel_scale is not None else G / 1000.0


def _imu_gyro_scale(_msg: str):
    # SCALED_IMU/RAW_IMU gyro fields are in millirad/s.
    return 0.001


def _calibrated_imu_accel(imu: dict):
    """
    Convert raw MAVLink IMU acceleration counts to m/s².

    SCALED_IMU and RAW_IMU are documented as milli-g, but some setups expose
    a static norm that is not exactly 1000 counts. During the initial/reset
    still period, normalize the measured static norm to 1g and freeze that
    scale for subsequent integration.
    """
    global _accel_scale, _accel_scale_samples

    x_count = float(imu.get("xacc", 0))
    y_count = float(imu.get("yacc", 0))
    z_count = float(imu.get("zacc", 0))
    norm_counts = math.sqrt(x_count**2 + y_count**2 + z_count**2)

    if _accel_scale is None and norm_counts > 1:
        _accel_scale_samples.append(norm_counts)
        avg_counts = sum(_accel_scale_samples) / len(_accel_scale_samples)
        scale = G / avg_counts
        if len(_accel_scale_samples) >= GRAVITY_CAL_SAMPLES:
            _accel_scale = scale
    else:
        scale = _imu_accel_scale("")

    ax = x_count * scale
    ay = y_count * scale
    az = z_count * scale
    return {
        "ax": ax,
        "ay": ay,
        "az": az,
        "norm": math.sqrt(ax**2 + ay**2 + az**2),
        "scale": scale,
        "scale_calibrated": _accel_scale is not None,
        "norm_counts": norm_counts,
    }


def _apply_deadband(v: float) -> float:
    return v if abs(v) >= ACCEL_DEADBAND else 0.0


def _clamp_delta(value: float, previous: float, limit: float) -> float:
    delta = value - previous
    if delta > limit:
        return previous + limit
    if delta < -limit:
        return previous - limit
    return value


def _filter_linear_accel(ax: float, ay: float, az: float):
    """
    Suppress single-frame spikes in world-frame linear acceleration, then apply
    an exponential moving average before the deadband and integration steps.
    """
    global _filtered_accel

    if _filtered_accel is None:
        _filtered_accel = (ax, ay, az)
        return _apply_deadband(ax), _apply_deadband(ay), _apply_deadband(az)

    px, py, pz = _filtered_accel
    ax = _clamp_delta(ax, px, ACCEL_JUMP_LIMIT)
    ay = _clamp_delta(ay, py, ACCEL_JUMP_LIMIT)
    az = _clamp_delta(az, pz, ACCEL_JUMP_LIMIT)

    alpha = ACCEL_FILTER_ALPHA
    fx = px + alpha * (ax - px)
    fy = py + alpha * (ay - py)
    fz = pz + alpha * (az - pz)
    _filtered_accel = (fx, fy, fz)

    return _apply_deadband(fx), _apply_deadband(fy), _apply_deadband(fz)


def _axis_component(axis: str, accel: dict) -> float:
    axis = axis.strip().lower()
    sign = -1.0 if axis.startswith("-") else 1.0
    name = axis[1:] if axis.startswith("-") else axis
    if name == "x":
        return sign * accel["ax"]
    if name == "y":
        return sign * accel["ay"]
    if name == "z":
        return sign * accel["az"]
    return 0.0


def _imu_to_body_accel(accel: dict):
    """
    Map raw IMU sensor axes into the vehicle body axes used by ATTITUDE.
    Default x,z,y matches the observed SCALED_IMU2 static gravity on raw Y.
    Override with IMU_BODY_AXIS_MAP, e.g. "x,y,z" or "x,z,-y".
    """
    axes = [part.strip() for part in IMU_BODY_AXIS_MAP.split(",")]
    if len(axes) != 3:
        axes = ["x", "z", "y"]
    return (
        _axis_component(axes[0], accel),
        _axis_component(axes[1], accel),
        _axis_component(axes[2], accel),
    )


def _linear_accel_from_gravity_bias(ax_g: float, ay_g: float, az_g: float):
    """
    Remove the still gravity/bias baseline from the rotated acceleration.
    Returns linear acceleration and whether the baseline is ready.
    """
    global _gravity_bias, _gravity_samples

    if _gravity_bias is None:
        _gravity_samples.append((ax_g, ay_g, az_g))
        if len(_gravity_samples) >= GRAVITY_CAL_SAMPLES:
            n = len(_gravity_samples)
            _gravity_bias = (
                sum(p[0] for p in _gravity_samples) / n,
                sum(p[1] for p in _gravity_samples) / n,
                sum(p[2] for p in _gravity_samples) / n,
            )
        return 0.0, 0.0, 0.0, False

    ax = ax_g - _gravity_bias[0]
    ay = ay_g - _gravity_bias[1]
    az = az_g - _gravity_bias[2]
    return ax, ay, az, True


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

def _ned_world_to_enu(ax_n, ay_n, az_n):
    """世界系 NED(北,东,下) → ENU(东,北,上)，与前端 X东/Y北/Z天 标注一致。"""
    return ay_n, ax_n, -az_n


def _append_track(x, y, z, attitude, ahrs2, spd, running_time):
    """按 DR_HZ/2 降采样记录一个轨迹点（ENU 坐标）。"""
    if int(running_time * DR_HZ) % 2 != 0:
        return
    depth = ahrs2.get("altitude", 0) if ahrs2 else 0.0
    point = {
        "ts":    time.time(),
        "x":     round(x, 3),
        "y":     round(y, 3),
        "z":     round(z, 3),
        "depth": round(depth, 3),
        "roll":  round(attitude.get("roll", 0), 4) if attitude else 0.0,
        "pitch": round(attitude.get("pitch", 0), 4) if attitude else 0.0,
        "yaw":   round(attitude.get("yaw", 0), 4) if attitude else 0.0,
        "speed": round(spd, 3),
    }
    with _track_lock:
        _track.append(point)


# ── 航位推算主循环 ────────────────────────────────────────────────────────────

def _dr_loop():
    global _gravity_bias, _gravity_samples, _accel_scale, _accel_scale_samples, _filtered_accel
    last_ts     = None        # 上一圈墙钟时刻，用于计算真实 dt
    imu_pref    = None        # 锁定已成功的 IMU 消息源
    slow_i      = 0           # 低频遥测计数：AHRS2/EKF 每 5 圈拉一次
    cache_ahrs2 = None
    cache_ekf   = None
    ekf_origin  = None        # 首次/重置时的 EKF 位置基准（ENU）

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
            _accel_scale = None
            _accel_scale_samples = []
            _filtered_accel = None
            last_ts = None
            imu_pref = None
            ekf_origin = None

        t0 = time.time()

        # ── 真实 dt（替代固定 dt，避免拉取耗时抖动导致积分失真） ───────────
        now = time.time()
        first = last_ts is None
        dt = 1.0 / DR_HZ if first else min(max(now - last_ts, DT_MIN), DT_MAX)
        last_ts = now

        # ── 拉取：姿态 + IMU（锁定有效源）+ EKF 融合位置（主） ─────────────
        attitude = _fetch("ATTITUDE")
        imu_msg, imu = _fetch_imu(imu_pref)
        if imu:
            imu_pref = imu_msg
        lpos = _fetch("LOCAL_POSITION_NED")
        accel = _calibrated_imu_accel(imu) if imu else None

        # ── 低频遥测：AHRS2 / EKF 方差（每 5 圈一次，减少阻塞拉取） ────────
        slow_i = (slow_i + 1) % 5
        if first or slow_i == 0:
            cache_ahrs2 = _fetch("AHRS2")
            cache_ekf = _fetch("EKF_STATUS_REPORT")
        ahrs2, ekf = cache_ahrs2, cache_ekf

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
            ax_body, ay_body, az_body = _imu_to_body_accel(accel)
            telem_update["ax_raw"] = round(accel["ax"], 4)
            telem_update["ay_raw"] = round(accel["ay"], 4)
            telem_update["az_raw"] = round(accel["az"], 4)
            telem_update["ax_body"] = round(ax_body, 4)
            telem_update["ay_body"] = round(ay_body, 4)
            telem_update["az_body"] = round(az_body, 4)
            telem_update["acc_norm_raw"] = round(accel["norm"], 4)
            telem_update["accel_scale"] = round(accel["scale"], 8)
            telem_update["accel_scale_calibrated"] = accel["scale_calibrated"]
            telem_update["accel_norm_counts"] = round(accel["norm_counts"], 2)
            telem_update["imu_body_axis_map"] = IMU_BODY_AXIS_MAP
            gyro_scale = _imu_gyro_scale(imu_msg)
            telem_update["gx_raw"] = round(imu.get("xgyro", 0) * gyro_scale, 5)
            telem_update["gy_raw"] = round(imu.get("ygyro", 0) * gyro_scale, 5)
            telem_update["gz_raw"] = round(imu.get("zgyro", 0) * gyro_scale, 5)

        with _telem_lock:
            _telem.update(telem_update)

        # ── 位置解算：优先自驾仪 EKF 融合位置，缺失时退回纯 IMU 积分 ──────
        if lpos is not None:
            # 主：LOCAL_POSITION_NED（已融合，无 t² 漂移），NED→ENU 并相对重置点输出
            e  = lpos.get("y", 0.0);  n  = lpos.get("x", 0.0);  u  = -lpos.get("z", 0.0)
            ve = lpos.get("vy", 0.0); vn = lpos.get("vx", 0.0); vu = -lpos.get("vz", 0.0)
            if ekf_origin is None:
                ekf_origin = (e, n, u)
            x, y, z = e - ekf_origin[0], n - ekf_origin[1], u - ekf_origin[2]
            spd = math.sqrt(ve * ve + vn * vn + vu * vu)
            with _dr_lock:
                px, py, pz = _dr_state["x"], _dr_state["y"], _dr_state["z"]
                _dr_state.update(x=x, y=y, z=z, vx=ve, vy=vn, vz=vu, drift_warn=False)
                _dr_state["dist"] += math.sqrt((x - px) ** 2 + (y - py) ** 2 + (z - pz) ** 2)
                _dr_state["running_time"] += dt
                dist = _dr_state["dist"]
                rt = _dr_state["running_time"]
            with _telem_lock:
                _telem.update({
                    "dr_x": round(x, 3), "dr_y": round(y, 3), "dr_z": round(z, 3),
                    "dr_dist": round(dist, 2), "dr_speed": round(spd, 3),
                    "dr_time": round(rt, 1), "drift_warn": False, "dr_source": "ekf",
                })
            _append_track(x, y, z, attitude, ahrs2, spd, rt)

        elif accel and attitude:
            # 兜底：纯 IMU 捷联积分（真实 dt + 静止在线零偏重估 + ZUPT），输出 ENU
            roll  = attitude.get("roll",  0)
            pitch = attitude.get("pitch", 0)
            yaw   = attitude.get("yaw",   0)

            ax_body, ay_body, az_body = _imu_to_body_accel(accel)
            ax_g, ay_g, az_g = _body_to_world(ax_body, ay_body, az_body, roll, pitch, yaw)
            ax_lin, ay_lin, az_lin, _ready = _linear_accel_from_gravity_bias(ax_g, ay_g, az_g)
            ax_n, ay_n, az_n = _filter_linear_accel(ax_lin, ay_lin, az_lin)
            ae, an, au = _ned_world_to_enu(ax_n, ay_n, az_n)   # 世界系 NED→ENU

            with _dr_lock:
                sf = _dr_state["still_frames"]
                acc_mag = math.sqrt(ae * ae + an * an + au * au)

                # 静止检测：加速度持续很小则计为静止
                if acc_mag < STILL_THRESH:
                    sf += 1
                else:
                    sf = 0

                if sf >= STILL_COUNT:
                    _dr_state["vx"] = 0.0
                    _dr_state["vy"] = 0.0
                    _dr_state["vz"] = 0.0
                    # 静止窗口是免费的在线标定：缓慢把重力/零偏基线拉向当前静止读数
                    if _gravity_bias is not None:
                        a = STILL_BIAS_ALPHA
                        _gravity_bias = (
                            _gravity_bias[0] + a * (ax_g - _gravity_bias[0]),
                            _gravity_bias[1] + a * (ay_g - _gravity_bias[1]),
                            _gravity_bias[2] + a * (az_g - _gravity_bias[2]),
                        )

                _dr_state["still_frames"] = sf

                # 速度 / 位置积分（真实 dt）
                _dr_state["vx"] += ae * dt
                _dr_state["vy"] += an * dt
                _dr_state["vz"] += au * dt
                dx = _dr_state["vx"] * dt
                dy = _dr_state["vy"] * dt
                dz = _dr_state["vz"] * dt
                _dr_state["x"] += dx
                _dr_state["y"] += dy
                _dr_state["z"] += dz
                _dr_state["dist"] += math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
                _dr_state["running_time"] += dt
                _dr_state["drift_warn"] = _dr_state["running_time"] > 60

                x, y, z = _dr_state["x"], _dr_state["y"], _dr_state["z"]
                dist = _dr_state["dist"]
                dw   = _dr_state["drift_warn"]
                rt   = _dr_state["running_time"]
                spd  = math.sqrt(_dr_state["vx"]**2 + _dr_state["vy"]**2 + _dr_state["vz"]**2)

            with _telem_lock:
                _telem.update({
                    "dr_x": round(x, 3), "dr_y": round(y, 3), "dr_z": round(z, 3),
                    "dr_dist": round(dist, 2), "dr_speed": round(spd, 3),
                    "dr_time": round(rt, 1), "drift_warn": dw, "dr_source": "imu",
                    "ax_w": round(ae, 4), "ay_w": round(an, 4), "az_w": round(au, 4),
                    "ax_w_raw": round(ax_lin, 4), "ay_w_raw": round(ay_lin, 4), "az_w_raw": round(az_lin, 4),
                    "ax_g": round(ax_g, 4), "ay_g": round(ay_g, 4), "az_g": round(az_g, 4),
                    "accel_filter_alpha": ACCEL_FILTER_ALPHA,
                    "accel_jump_limit": ACCEL_JUMP_LIMIT,
                    "gravity_calibrated": _gravity_bias is not None,
                    "gravity_cal_samples": len(_gravity_samples),
                    "gravity_bias": [round(v, 4) for v in _gravity_bias] if _gravity_bias else None,
                })
            _append_track(x, y, z, attitude, ahrs2, spd, rt)

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
