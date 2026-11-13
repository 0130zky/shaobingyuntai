"""Serial I/O boundary. POSIX stays stdlib-only; Windows uses optional pyserial."""
import os
import struct
import sys
import time
if os.name != 'nt':
    import fcntl
    import termios
    import select

class _PosixSerialPort:
    def __init__(self, path, baud):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.old = None
        self.exclusive = False
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if hasattr(termios, 'TIOCEXCL'):
                fcntl.ioctl(self.fd, termios.TIOCEXCL)
                self.exclusive = True
            self.old = termios.tcgetattr(self.fd)
            cfg = termios.tcgetattr(self.fd)
            cfg[0] = cfg[1] = cfg[3] = 0
            cfg[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            speed = getattr(termios, 'B%d' % baud, None)
            cfg[4] = cfg[5] = speed if speed is not None else termios.B115200
            cfg[6][termios.VMIN] = cfg[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, cfg)
            if speed is None:
                if sys.platform != 'darwin':
                    raise ValueError('Unsupported baud rate: %d' % baud)
                packed = struct.pack('@L', baud)
                request = 0x80000000 | (len(packed) << 16) | (ord('T') << 8) | 2
                fcntl.ioctl(self.fd, request, packed)
            termios.tcflush(self.fd, termios.TCIFLUSH)
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.fd is None:
            return
        try:
            if self.old is not None:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.old)
        except OSError:
            pass
        try:
            if self.exclusive and hasattr(termios, 'TIOCNXCL'):
                fcntl.ioctl(self.fd, termios.TIOCNXCL)
        except OSError:
            pass
        os.close(self.fd)
        self.fd = None

    def read(self, size=65536, timeout=.1):
        if not select.select([self.fd], [], [], timeout)[0]:
            return b''
        data = os.read(self.fd, size)
        if not data:
            raise OSError('Serial device disconnected')
        return data

    def write(self, data):
        return os.write(self.fd, data)


class _PySerialPort:
    def __init__(self, path, baud):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError('Install tools/requirements.txt for Windows serial support') from exc
        try:
            self.port = serial.serial_for_url(path, baudrate=baud, timeout=.1, write_timeout=.5)
        except serial.SerialException as exc:
            raise OSError(str(exc)) from exc

    def read(self, size=65536, timeout=.1):
        self.port.timeout = timeout
        try:
            return self.port.read(min(size, max(1, self.port.in_waiting)))
        except OSError:
            raise
        except Exception as exc:
            raise OSError(str(exc)) from exc

    def write(self, data):
        try:
            return self.port.write(data)
        except Exception as exc:
            raise OSError(str(exc)) from exc

    def close(self):
        self.port.close()


def SerialPort(path, baud):
    return _PySerialPort(path, baud) if os.name == 'nt' or '://' in path else _PosixSerialPort(path, baud)


def available_ports():
    try:
        from serial.tools.list_ports import comports
        return sorted(port.device for port in comports())
    except ImportError:
        import glob
        ports = glob.glob('/dev/cu.usbmodem*') or glob.glob('/dev/serial/by-id/*')
        return sorted(ports or (glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')))
