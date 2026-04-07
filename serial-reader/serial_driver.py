"""
串口驱动核心模块
负责：打开串口、读取数据、维护历史记录、线程管理
"""

import threading
import time
from collections import deque
from datetime import datetime
from typing import Optional

import serial
import serial.tools.list_ports
from loguru import logger

MAX_HISTORY = 50000
DEFAULT_PORT = "/dev/ttyAMA0"
#DEFAULT_PORT = "/dev/serial1"
DEFAULT_BAUD = 115200


class SerialDriver:
    def __init__(self):
        self.port: str = DEFAULT_PORT
        self.baud: int = DEFAULT_BAUD
        self.connected: bool = False
        self.error: Optional[str] = None
        self.enabled: bool = True

        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self.history: deque = deque(maxlen=MAX_HISTORY)
        self.total_bytes: int = 0
        self.total_lines: int = 0
        self.start_time: float = time.time()

    # ── 公开接口 ──────────────────────────────────────────────────────────────

    def start(self):
        """启动后台读取线程"""
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        logger.info("Serial reader thread started")

    def get_status(self) -> dict:
        return {
            "connected":   self.connected,
            "enabled":     self.enabled,
            "port":        self.port,
            "baud":        self.baud,
            "error":       self.error,
            "total_lines": self.total_lines,
            "total_bytes": self.total_bytes,
            "uptime":      round(time.time() - self.start_time, 1),
            "history":     list(self.history),
        }

    def set_port(self, port: str) -> bool:
        with self._lock:
            self.port = port
            self._close()
        logger.info(f"Port changed to {port}")
        return True

    def set_baud(self, baud: int) -> bool:
        with self._lock:
            self.baud = baud
            self._close()
        logger.info(f"Baud changed to {baud}")
        return True

    def set_enabled(self, enabled: bool) -> bool:
        self.enabled = enabled
        if not enabled:
            self._close()
        logger.info(f"Driver {'enabled' if enabled else 'disabled'}")
        return True

    def clear_history(self):
        self.history.clear()
        logger.info("History cleared")

    def list_ports(self) -> list:
        return [
            {"device": p.device, "description": p.description}
            for p in serial.tools.list_ports.comports()
        ]

    # ── 内部实现 ──────────────────────────────────────────────────────────────

    def _close(self):
        """关闭当前串口连接"""
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        self.connected = False

    def _reader_loop(self):
        """后台线程：持续尝试打开串口并读取数据"""
        while True:
            if not self.enabled:
                time.sleep(1)
                continue

            # 尝试打开串口
            try:
                with self._lock:
                    self._serial = serial.Serial(
                        port=self.port,
                        baudrate=self.baud,
                        timeout=1.0,
                    )
                self.connected = True
                self.error = None
                logger.info(f"Opened {self.port} @ {self.baud}")
            except Exception as e:
                self.connected = False
                self.error = str(e)
                logger.warning(f"Cannot open {self.port}: {e}")
                time.sleep(3)
                continue

            # 读取循环
            try:
                while self.enabled:
                    raw_bytes = self._serial.readline()
                    if not raw_bytes:
                        continue

                    self.total_bytes += len(raw_bytes)

                    try:
                        text = raw_bytes.decode("utf-8", errors="replace").strip()
                    except Exception:
                        text = repr(raw_bytes)

                    if text:
                        self.total_lines += 1
                        entry = {
                            "index":     self.total_lines,
                            "timestamp": datetime.now().strftime("%H:%M:%S.%f")[:12],
                            "raw":       text,
                        }
                        self.history.append(entry)

            except Exception as e:
                self.connected = False
                self.error = str(e)
                logger.error(f"Serial read error: {e}")
                self._close()
                time.sleep(3)
