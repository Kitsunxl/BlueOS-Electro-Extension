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

MAX_HISTORY = 8000
DEFAULT_PORT = "/dev/ttyAMA0"
#DEFAULT_PORT = "/dev/serial1"
DEFAULT_BAUD = 115200
READ_TIMEOUT = 0.05
PARTIAL_FLUSH_SEC = 0.12


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
        self._history_lock = threading.Lock()

        self.history: deque = deque(maxlen=MAX_HISTORY)
        self.total_bytes: int = 0
        self.total_lines: int = 0
        self.start_time: float = time.time()
        self.last_rx_time: Optional[float] = None

    # ── 公开接口 ──────────────────────────────────────────────────────────────

    def start(self):
        """启动后台读取线程"""
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        logger.info("Serial reader thread started")

    def get_status(self) -> dict:
        with self._history_lock:
            history_size = len(self.history)

        return {
            "connected":   self.connected,
            "enabled":     self.enabled,
            "port":        self.port,
            "baud":        self.baud,
            "error":       self.error,
            "total_lines": self.total_lines,
            "total_bytes": self.total_bytes,
            "uptime":      round(time.time() - self.start_time, 1),
            "history_size": history_size,
            "last_rx_age":  round(time.time() - self.last_rx_time, 3) if self.last_rx_time else None,
        }

    def get_history_since(self, since_index: int, limit: int = 2000) -> list:
        limit = max(1, min(int(limit), 5000))
        with self._history_lock:
            snapshot = list(self.history)

        if since_index <= 0:
            return snapshot[-limit:] if len(snapshot) > limit else snapshot

        if not snapshot or since_index >= snapshot[-1]["index"]:
            return []

        first_index = snapshot[0]["index"]
        start = max(0, since_index - first_index + 1)
        return snapshot[start:start + limit]

    def get_all_history(self) -> list:
        with self._history_lock:
            return list(self.history)

    def export_history(self, limit: int = 30000) -> list:
        limit = max(1, min(int(limit), 30000))
        with self._history_lock:
            snapshot = list(self.history)
        if len(snapshot) > limit:
            return snapshot[-limit:]
        return snapshot

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
        with self._history_lock:
            self.history.clear()
        self.total_lines = 0
        self.total_bytes = 0
        self.last_rx_time = None
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
                        timeout=READ_TIMEOUT,
                        inter_byte_timeout=READ_TIMEOUT,
                    )
                    self._serial.reset_input_buffer()
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
                pending = bytearray()
                partial_since = None
                while self.enabled:
                    waiting = self._serial.in_waiting
                    raw_bytes = self._serial.read(waiting or 1)
                    if not raw_bytes:
                        if pending and partial_since and time.time() - partial_since >= PARTIAL_FLUSH_SEC:
                            self._append_line(bytes(pending))
                            pending.clear()
                            partial_since = None
                        continue

                    self.total_bytes += len(raw_bytes)
                    self.last_rx_time = time.time()
                    pending.extend(raw_bytes)
                    if partial_since is None:
                        partial_since = self.last_rx_time

                    while True:
                        newline_positions = [p for p in (pending.find(b"\n"), pending.find(b"\r")) if p >= 0]
                        if not newline_positions:
                            break
                        cut = min(newline_positions)
                        line = bytes(pending[:cut])
                        del pending[:cut + 1]
                        while pending[:1] in (b"\n", b"\r"):
                            del pending[:1]
                        self._append_line(line)
                        partial_since = time.time() if pending else None

            except Exception as e:
                self.connected = False
                self.error = str(e)
                logger.error(f"Serial read error: {e}")
                self._close()
                time.sleep(3)

    def _append_line(self, raw_bytes: bytes):
        try:
            text = raw_bytes.decode("utf-8", errors="replace").strip()
        except Exception:
            text = repr(raw_bytes)

        if not text:
            return

        self.total_lines += 1
        entry = {
            "index":     self.total_lines,
            "timestamp": datetime.now().strftime("%H:%M:%S.%f")[:12],
            "raw":       text,
        }
        with self._history_lock:
            self.history.append(entry)
