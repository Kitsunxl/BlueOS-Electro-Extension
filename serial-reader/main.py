#!/usr/bin/env python3
"""
BlueOS Serial Reader Extension
使用 Flask，完全参照 DVL 项目结构
"""

from flask import Flask
from serial_driver import SerialDriver

app = Flask(__name__, static_url_path="/static", static_folder="static")


class API:
    def __init__(self, driver: SerialDriver):
        self.driver = driver

    def get_status(self):
        return self.driver.get_status()

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

    driver.start()
    app.run(host="0.0.0.0", port=9001)
