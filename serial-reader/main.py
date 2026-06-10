#!/usr/bin/env python3
"""
BlueOS Serial Reader Extension
使用 Flask，完全参照 DVL 项目结构
"""

import json
import time
import threading
from collections import deque
from flask import Flask, request as flask_request
from serial_driver import SerialDriver
from blueoshelper import request as blueos_request

app = Flask(__name__, static_url_path="/static", static_folder="static")

# ── MAVLink 遥测数据采集 ────────────────────────────────────────────────────

MAVLINK_BASE = "http://127.0.0.1:6040/v1/mavlink/vehicles/1/components/1/messages"
MAX_TRACK_POINTS = 5000   # 最多保留轨迹点数

_track: deque = deque(maxlen=MAX_TRACK_POINTS)
_track_lock = threading.Lock()
_latest_telemetry: dict = {}
_telem_lock = threading.Lock()


def _fetch_mavlink(msg_name: str):
    """从 MAVLink2REST 获取单条消息，返回 message 字段字典或 None"""
    try:
        raw = blueos_request(f"{MAVLINK_BASE}/{msg_name}")
        if raw is None:
            return None
        data = json.loads(raw)
        return data.get("message", {})
    except Exception:
        return None


def _telemetry_loop():
    """后台线程：以约 5Hz 采集姿态 + 位置数据，追加轨迹点"""
    while True:
        try:
            attitude = _fetch_mavlink("ATTITUDE")
            ahrs2    = _fetch_mavlink("AHRS2")
            pos      = _fetch_mavlink("GLOBAL_POSITION_INT")
            ekf      = _fetch_mavlink("EKF_STATUS_REPORT")

            telem = {
                "ts": time.time(),
            }

            if attitude:
                telem["roll"]  = round(attitude.get("roll",  0), 4)
                telem["pitch"] = round(attitude.get("pitch", 0), 4)
                telem["yaw"]   = round(attitude.get("yaw",   0), 4)

            if ahrs2:
                telem["ahrs2_roll"]  = round(ahrs2.get("roll",  0), 4)
                telem["ahrs2_pitch"] = round(ahrs2.get("pitch", 0), 4)
                telem["ahrs2_yaw"]   = round(ahrs2.get("yaw",   0), 4)
                telem["altitude"]    = round(ahrs2.get("altitude", 0), 3)

            if pos:
                # GLOBAL_POSITION_INT 单位：lat/lon 1e-7 度，alt 毫米
                lat = pos.get("lat", 0)
                lon = pos.get("lon", 0)
                alt_mm = pos.get("alt", 0)
                rel_alt_mm = pos.get("relative_alt", 0)
                vx  = pos.get("vx", 0)   # cm/s
                vy  = pos.get("vy", 0)
                vz  = pos.get("vz", 0)
                hdg = pos.get("hdg", 0)  # cdeg

                telem["lat"]     = lat / 1e7
                telem["lon"]     = lon / 1e7
                telem["alt_m"]   = alt_mm / 1000.0
                telem["rel_alt"] = rel_alt_mm / 1000.0
                telem["vx"]      = vx / 100.0
                telem["vy"]      = vy / 100.0
                telem["vz"]      = vz / 100.0
                telem["heading"] = hdg / 100.0

            if ekf:
                telem["ekf_flags"]        = ekf.get("flags", {})
                telem["ekf_vel_variance"] = round(ekf.get("velocity_variance", 0), 4)
                telem["ekf_pos_horiz"]    = round(ekf.get("pos_horiz_variance", 0), 4)
                telem["ekf_pos_vert"]     = round(ekf.get("pos_vert_variance", 0), 4)

            with _telem_lock:
                _latest_telemetry.update(telem)

            # 只在有有效位置时记录轨迹点
            if pos and (telem.get("lat", 0) != 0 or telem.get("lon", 0) != 0):
                point = {
                    "ts":      telem["ts"],
                    "lat":     telem.get("lat", 0),
                    "lon":     telem.get("lon", 0),
                    "alt":     telem.get("rel_alt", telem.get("altitude", 0)),
                    "heading": telem.get("heading", 0),
                    "roll":    telem.get("roll",  0),
                    "pitch":   telem.get("pitch", 0),
                    "yaw":     telem.get("yaw",   0),
                }
                with _track_lock:
                    _track.append(point)

        except Exception:
            pass

        time.sleep(0.2)   # 5 Hz


# 启动遥测采集线程
_telem_thread = threading.Thread(target=_telemetry_loop, daemon=True)
_telem_thread.start()


class API:
    def __init__(self, driver: SerialDriver):
        self.driver = driver

    def get_status(self):
        return self.driver.get_status()

    def get_history_since(self, since: int, limit: int = 2000) -> list:
        return self.driver.get_history_since(since, limit)

    def export_history(self, limit: int = 30000) -> list:
        return self.driver.export_history(limit)

    def set_enabled(self, enabled: str) -> bool:
        if enabled in ["true", "false"]:
            return self.driver.set_enabled(enabled == "true")
        return False

    def set_port(self, port: str) -> bool:
        return self.driver.set_port(port)

    def set_baud(self, baud: str) -> bool:
        try:
            return self.driver.set_baud(int(baud))
        except ValueError:
            return False

    def clear_history(self) -> bool:
        self.driver.clear_history()
        return True

    def list_ports(self) -> list:
        return self.driver.list_ports()


if __name__ == "__main__":
    driver = SerialDriver()
    api = API(driver)

    # ── REST 路由，与 DVL 项目风格完全一致 ────────────────────────────────────

    @app.route("/register_service")
    def register_service():
        """BlueOS 通过此接口识别扩展，必须有"""
        return app.send_static_file("service.json")

    @app.route("/")
    def root():
        return app.send_static_file("index.html")

    @app.route("/get_status")
    def get_status():
        import json
        return json.dumps(api.get_status())

    @app.route("/get_history_since/<int:since>")
    def get_history_since(since: int):
        import json
        return json.dumps(api.get_history_since(since))

    @app.route("/export_history")
    def export_history():
        import json
        from flask import request

        try:
            limit = int(request.args.get("limit", 30000))
        except (TypeError, ValueError):
            limit = 30000

        return json.dumps(api.export_history(limit))

    @app.route("/enable/<enable>")
    def set_enabled(enable: str):
        return str(api.set_enabled(enable))

    @app.route("/set_port/<path:port>")
    def set_port(port: str):
        return str(api.set_port(port))

    @app.route("/set_baud/<baud>")
    def set_baud(baud: str):
        return str(api.set_baud(baud))

    @app.route("/clear_history")
    def clear_history():
        return str(api.clear_history())

    @app.route("/list_ports")
    def list_ports():
        import json
        return json.dumps(api.list_ports())

    # ── 遥测 & 轨迹路由 ───────────────────────────────────────────────────────

    @app.route("/get_telemetry")
    def get_telemetry():
        with _telem_lock:
            return json.dumps(_latest_telemetry)

    @app.route("/get_track")
    def get_track():
        """返回全部轨迹点"""
        with _track_lock:
            return json.dumps(list(_track))

    @app.route("/get_track_since/<since_ts>")
    def get_track_since(since_ts: str):
        """返回 since_ts 之后的轨迹点（增量拉取）"""
        try:
            ts = float(since_ts)
        except ValueError:
            ts = 0.0
        with _track_lock:
            result = [p for p in _track if p["ts"] > ts]
        return json.dumps(result)

    @app.route("/clear_track")
    def clear_track():
        with _track_lock:
            _track.clear()
        return json.dumps(True)

    driver.start()
    app.run(host="0.0.0.0", port=9001)
