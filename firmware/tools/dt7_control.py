"""DT7 USB keyboard relay, independent of the board telemetry serial port."""
import struct
import threading
import time
from serial_transport import SerialPort


def crc(data, initial, polynomial):
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value


def packet(keys=0, mouse=(0, 0, 0), buttons=(0, 0)):
    if not isinstance(keys, int) or not 0 <= keys <= 65535:
        raise ValueError('键盘掩码超出范围')
    data = bytearray(b'\x5a\x10\x01')
    data.append(crc(data, 0x77, 0x8c))
    data.extend(struct.pack('<HHhhhBBHH', 0, 0x2000, *mouse, *buttons, keys, 0))
    data.extend(struct.pack('<H', crc(data, 0x1862, 0x8408)))
    return bytes(data)


class DT7Control:
    """Only explicit connect writes. UI heartbeat expires after 250 ms.

    X (bit14) identifies this panel; B (bit15) toggles every 84ms so the
    receiver can reject a frozen last keyboard state even after process death.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._serial = None
        self._thread = None
        self._stop = threading.Event()
        self.port = None
        self.error = None
        self.keys = 0
        self.updated = 0
        self.frames = 0

    def connect(self, port):
        self.disconnect()
        if not port:
            raise ValueError('请选择 DT7 USB 串口')
        serial = SerialPort(port, 921600)
        with self._lock:
            self._serial = serial
            self.port, self.error, self.frames = port, None, 0
            self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name='dt7-relay')
        self._thread.start()

    def update(self, keys):
        if type(keys) is not int or keys & ~15 or keys < 0:
            raise ValueError('仅支持 W/S/A/D')
        with self._lock:
            if self._serial is None:
                raise ValueError('DT7 未连接')
            self.keys, self.updated = keys, time.monotonic()

    def snapshot(self):
        with self._lock:
            return dict(connected=self._serial is not None, port=self.port,
                        error=self.error, frames=self.frames,
                        keys=self.keys if time.monotonic()-self.updated < .25 else 0)

    @staticmethod
    def _write(serial, data):
        if serial.write(data) != len(data):
            raise OSError('DT7 串口未完整写入')

    def _run(self):
        serial = self._serial
        stop = self._stop
        started = time.monotonic()
        try:
            while not stop.is_set():
                now = time.monotonic()
                with self._lock:
                    keys = self.keys if now-self.updated < .25 else 0
                # Neutral means truly all-zero: release cancels an active sequence.
                wire = keys | 0x4000 | ((int((now-started)/.084) & 1) << 15) if keys else 0
                self._write(serial, packet(wire))
                with self._lock:
                    self.frames += 1
                stop.wait(.014)
        except (OSError, ValueError) as exc:
            with self._lock:
                self.error = str(exc)
        finally:
            try:
                for _ in range(3):
                    self._write(serial, packet())
            except (OSError, ValueError):
                pass
            serial.close()
            with self._lock:
                self._serial = None
                self.port = None
                self.keys = 0

    def disconnect(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
            if self._thread.is_alive():
                raise ValueError('DT7 发送线程尚未结束')
            self._thread = None
