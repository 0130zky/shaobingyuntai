#!/usr/bin/env python3
"""VS Code 折叠云台编码器上位机后台。

数据源是 DM-MC02 的 USB CDC：Sentry_Background() 每 100ms 输出一行

    SENTRY seq=... fold_raw=... fold_mrad=... pitch_raw=... pitch_mrad=...

板端串口只解析状态行；独立 DT7 串口显式连接后发送键盘中转报文。VS Code 扩展通过
stdin/stdout 的 JSON RPC 读取 state/connect/disconnect/ports。
"""
import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from serial_transport import SerialPort, available_ports
from dt7_control import DT7Control

BAUD = 115200
RAW_MAX = 65535
P_MAX_RAD = 12.5          # robots/sentry/gimbal_config.h: SENTRY_P_MAX
HISTORY = 1800            # 10 Hz × 180 s
KEEP_LINE = 4096
FIRMWARE_DIR = Path(__file__).resolve().parent.parent
BUILD_SCRIPT = FIRMWARE_DIR / 'tools/sentry_build_flash.sh'
JOB_MODES = ('build', 'flash', 'program')
JOB_LOG = 600
JOB_TAIL = 240
RECONNECT_S = 15.0
PHASE_NAMES = {'build': '编译', 'flash': '烧录', 'done': '结束'}
SCALAR_FIELDS = (
    'keys', 'seq', 'result', 'cal', 'valid', 'mask', 'ready', 'rc', 'master', 'armed',
    'fault', 'fold_state', 'fold_raw', 'fold_mrad', 'fold_goal', 'fold_age',
    'pitch_raw', 'pitch_mrad', 'pitch_goal', 'pitch_age', 's1', 's2', 'z_mode',
    'level_raw', 'level_err', 'fold_pitch_target', 'open_pitch_target',
)
PAIR_FIELDS = ('err', 'temp', 'sticks')
QUAD_FIELDS = ('ends', 'end_raw')


def raw_from_mrad(mrad):
    """MIT 位置量程是 ±12.5 rad，编码值 0..65535 对应该区间。"""
    return (mrad / 1000.0 + P_MAX_RAD) / (2.0 * P_MAX_RAD) * RAW_MAX


def _int(text):
    return int(text, 10)


def _ints(text, count):
    parts = text.split(',')
    if len(parts) != count:
        raise ValueError(text)
    return [_int(part) for part in parts]


def _axis(pairs, name, err, temp):
    raw = _int(pairs[name + '_raw'])
    if not 0 <= raw <= RAW_MAX:
        raise ValueError('raw')
    mrad = _int(pairs[name + '_mrad'])
    if abs(mrad) > 20000:
        raise ValueError('mrad')
    goal_mrad = _int(pairs[name + '_goal']) if name + '_goal' in pairs else None
    age_ms = _int(pairs[name + '_age']) if name + '_age' in pairs else None
    if age_ms is not None and not 0 <= age_ms <= 0xFFFFFFFF:
        raise ValueError('age')
    goal_raw = None
    error_raw = None
    if goal_mrad is not None and abs(goal_mrad) <= 20000:
        goal_raw = min(RAW_MAX, max(0, int(round(raw_from_mrad(goal_mrad)))))
        error_raw = raw - goal_raw
    return {
        'raw': raw,
        'mrad': mrad,
        'deg': mrad / 1000.0 * 180.0 / math.pi,
        'goal_mrad': goal_mrad,
        'goal_raw': goal_raw,
        'goal_deg': None if goal_mrad is None else goal_mrad / 1000.0 * 180.0 / math.pi,
        'error_raw': error_raw,
        'age_ms': age_ms,
        'seen': age_ms is not None and age_ms != 0xFFFFFFFF,
        'error_code': err,
        'temp_c': temp,
    }


def parse_status_line(line):
    """把一行 USB 状态解析成样本；格式不符返回 None，绝不猜测。"""
    try:
        text = line.decode('ascii').strip()
    except UnicodeDecodeError:
        return None
    if not text.startswith('SENTRY '):
        return None
    pairs = {}
    for token in text.split():
        if '=' not in token:
            if token != 'SENTRY':
                return None
            continue
        key, value = token.split('=', 1)
        if not key or key in pairs:
            return None
        pairs[key] = value
    for required in ('fold_raw', 'fold_mrad', 'pitch_raw', 'pitch_mrad'):
        if required not in pairs:
            return None
    try:
        values = {key: _int(pairs[key]) for key in SCALAR_FIELDS if key in pairs}
        pairs_parsed = {key: _ints(pairs[key], 2) for key in PAIR_FIELDS if key in pairs}
        quads = {key: _ints(pairs[key], 4) for key in QUAD_FIELDS if key in pairs}
        error = pairs_parsed.get('err', (None, None))
        temp = pairs_parsed.get('temp', (None, None))
        sample = {
            'received_unix_s': time.time(),
            'fold': _axis(pairs, 'fold', error[0], temp[0]),
            'pitch': _axis(pairs, 'pitch', error[1], temp[1]),
            'end_raw': quads.get('end_raw'),
            'ends': quads.get('ends'),
            'sticks': pairs_parsed.get('sticks'),
        }
        if 'mouse' in pairs:
            sample['mouse'] = _ints(pairs['mouse'], 3)
        sample.update(values)
        return sample
    except (KeyError, ValueError):
        return None


def _port_rank(path):
    name = path.lower()
    if 'usbmodem' in name or 'usbserial' in name:
        return (0, name)
    if 'acm' in name:
        return (1, name)
    if name.startswith('com'):
        return (2, name)
    if 'bluetooth' in name or 'debug-console' in name:
        return (8, name)
    return (5, name)


class BuildJob:
    """运行 tools/sentry_build_flash.sh，逐行保存输出供面板显示。

    脚本以 [build] / [flash] 前缀标注阶段；这里只解析前缀，不重新实现编译或烧录。
    """

    def __init__(self, mode, session, dry_run=False):
        self.mode = mode
        self.dry_run = dry_run
        self.session = session
        self.running = True
        self.phase = 'flash' if mode == 'program' else 'build'
        self.exit_code = None
        self.error = None
        self.started = time.time()
        self.finished = None
        self.lines = deque(maxlen=JOB_LOG)
        self._proc = None
        self._cancel = threading.Event()
        self._thread = threading.Thread(target=self._run, name='gimbal-build', daemon=True)
        self._thread.start()

    def record(self, text, phase=None):
        for line in str(text).splitlines():
            line = line.rstrip()
            if not line:
                continue
            if phase is None:
                for name in ('build', 'flash'):
                    if line.startswith(f'[{name}]'):
                        phase = name
            if phase:
                self.phase = phase
            self.lines.append({'t': time.strftime('%H:%M:%S'), 'text': line})

    def cancel(self):
        self._cancel.set()
        sequence = self._proc
        if sequence is not None and sequence.poll() is None:
            sequence.terminate()

    def _run(self):
        try:
            if self.mode in ('flash', 'program') and not self.dry_run:
                self.session.disconnect()
                self.record('[flash] 已断开串口，烧录后自动重连', 'flash')
            command = [str(BUILD_SCRIPT), self.mode]
            if self.dry_run:
                command.append('--dry-run')
            self.record(f'$ {" ".join(command)}', self.phase)
            env = dict(os.environ, NO_COLOR='1', CLICOLOR='0', TERM='dumb',
                       CMAKE_COLOR_DIAGNOSTICS='OFF', PYTHONUTF8='1')
            self._proc = subprocess.Popen(
                command, cwd=str(FIRMWARE_DIR), stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace',
                bufsize=1, env=env)
            for line in self._proc.stdout:
                self.record(line)
                if self._cancel.is_set():
                    self._proc.terminate()
                    break
            self.exit_code = self._proc.wait()
        except (OSError, ValueError) as exc:
            self.error = str(exc)
            self.exit_code = -1
        finally:
            if self._proc is not None and self._proc.stdout is not None:
                try:
                    self._proc.stdout.close()
                except OSError:
                    pass
            if self._cancel.is_set() and not self.error:
                self.error = '已取消'
            # Reconnecting is part of the flash step, so finish after the port is back.
            if self.mode == 'flash' and self.exit_code == 0 and not self.dry_run:
                self.session.reconnect(log=self.record)
            self.running = False
            self.finished = time.time()
            self.phase = 'done'

    def snapshot(self):
        end = self.finished or time.time()
        return {
            'mode': self.mode,
            'dry_run': self.dry_run,
            'running': self.running,
            'phase': self.phase,
            'phase_label': PHASE_NAMES.get(self.phase, self.phase),
            'exit_code': self.exit_code,
            'error': self.error,
            'duration_s': max(0.0, end - self.started),
            'output': list(self.lines)[-JOB_TAIL:],
        }


class EncoderSession:
    def __init__(self, auto_port=''):
        self._lock = threading.Lock()
        self._stop = None
        self._thread = None
        self._serial = None
        self.connected = False
        self.port = None
        self.error = None
        self.latest = None
        self.history = deque(maxlen=HISTORY)
        self.received = 0
        self.invalid = 0
        self.job = None
        self.dt7 = DT7Control()
        self._ports = []
        self.refresh_ports()
        if auto_port:
            try:
                self.connect(auto_port)
            except (OSError, ValueError) as exc:
                self.error = str(exc)

    def refresh_ports(self):
        self._ports = sorted(available_ports(), key=_port_rank)
        return list(self._ports)

    def connect(self, path=''):
        self.disconnect()
        path = (path or '').strip()
        if not path:
            candidates = [item for item in self._ports if _port_rank(item)[0] < 5
                          and '0001A0000000' not in item and '507211265703' not in item]
            path = candidates[0] if candidates else ''
        if not path:
            raise ValueError('未找到 USB 串口；连接开发板后点“刷新”')
        if path == self.dt7.snapshot()['port']:
            raise ValueError('此串口正用于 DT7 控制')
        serial_port = SerialPort(path, BAUD)
        stop = threading.Event()
        with self._lock:
            self._serial = serial_port
            self.port = path
            self.connected = True
            self.error = None
            self.latest = None
            self.history.clear()
            self.received = 0
            self.invalid = 0
            self._stop = stop
        self._thread = threading.Thread(target=self._read_loop, args=(serial_port, stop),
                                        name='gimbal-encoder', daemon=True)
        self._thread.start()
        return self.state()

    def disconnect(self):
        with self._lock:
            stop, serial_port = self._stop, self._serial
            self._stop = None
            self._serial = None
            self.connected = False
            self.port = None
        if stop:
            stop.set()
        if serial_port:
            try:
                serial_port.close()
            except OSError:
                pass
        thread, self._thread = self._thread, None
        if thread and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    def _read_loop(self, serial_port, stop):
        buffer = bytearray()
        error = None
        try:
            while not stop.is_set():
                try:
                    chunk = serial_port.read(timeout=.1)
                except OSError as exc:
                    error = str(exc) or '串口已断开'
                    break
                if not chunk:
                    continue
                buffer.extend(chunk)
                if len(buffer) > KEEP_LINE:
                    del buffer[:-KEEP_LINE]
                while True:
                    index = buffer.find(b'\n')
                    if index < 0:
                        break
                    line = bytes(buffer[:index])
                    del buffer[:index + 1]
                    sample = parse_status_line(line)
                    with self._lock:
                        if sample is None:
                            if line.strip():
                                self.invalid += 1
                        else:
                            self.latest = sample
                            self.received += 1
                            self.history.append({
                                't': sample['received_unix_s'],
                                'fold_raw': sample['fold']['raw'],
                                'pitch_raw': sample['pitch']['raw'],
                                'fold_goal_raw': sample['fold']['goal_raw'],
                                'pitch_goal_raw': sample['pitch']['goal_raw'],
                            })
        finally:
            try:
                serial_port.close()
            except OSError:
                pass
            with self._lock:
                if self._serial is serial_port:
                    self._serial = None
                    self.connected = False
                    self.port = None
                    if error and not stop.is_set():
                        self.error = error

    def reconnect(self, timeout=RECONNECT_S, log=None):
        """烧录复位后 USB CDC 会重新枚举，端口名可能变化；轮询直到重新连上。"""
        note = log or (lambda text, phase=None: None)
        deadline = time.time() + timeout
        while time.time() < deadline:
            for path in self.refresh_ports():
                if (_port_rank(path)[0] >= 5 or '0001A0000000' in path
                        or '507211265703' in path or path == self.dt7.snapshot()['port']):
                    continue
                try:
                    self.connect(path)
                except (OSError, ValueError):
                    continue
                note(f'[flash] 已重新连接 {path}', 'flash')
                return True
            time.sleep(.5)
        self.error = '烧录完成，但没重新发现 USB 串口；给板子重新上电后点“刷新”'
        note(f'[flash] {self.error}', 'flash')
        return False

    def start_job(self, mode, dry_run=False):
        if mode not in JOB_MODES:
            raise ValueError(f'未知任务 {mode}')
        job = self.job
        if job is not None and job.running:
            raise ValueError('已有编译/烧录任务在运行，先取消或等待结束')
        if not BUILD_SCRIPT.exists():
            raise ValueError(f'找不到 {BUILD_SCRIPT}')
        self.dt7.update(0) if self.dt7.snapshot()['connected'] else None
        self.job = BuildJob(mode, self, dry_run=bool(dry_run))
        return self.state()

    def cancel_job(self):
        job = self.job
        if job is None or not job.running:
            raise ValueError('没有正在运行的编译/烧录任务')
        job.cancel()
        return self.state()

    def state(self):
        now = time.time()
        with self._lock:
            latest = self.latest
            history = list(self.history)[-600:]
            return {
                'connected': self.connected,
                'port': self.port,
                'baud': BAUD,
                'error': self.error,
                'received': self.received,
                'invalid': self.invalid,
                'data_age_s': None if latest is None else max(0.0, now - latest['received_unix_s']),
                'latest': latest,
                'history': history,
                'ports': list(self._ports),
                'config': {'raw_max': RAW_MAX, 'p_max_rad': P_MAX_RAD,
                           'stale_ms': 250},
                'dt7': self.dt7.snapshot(),
                'job': None if self.job is None else self.job.snapshot(),
            }

    def dispatch(self, method, params):
        if method == 'dt7_connect':
            port = params.get('port', '')
            if port == self.port:
                raise ValueError('此串口正用于板端遥测')
            self.dt7.connect(port)
            return self.state()
        if method == 'dt7_disconnect':
            self.dt7.disconnect()
            return self.state()
        if method == 'dt7_input':
            if self.job is not None and self.job.running:
                raise ValueError('编译烧录期间暂停控制')
            self.dt7.update(params.get('keys', 0))
            return self.dt7.snapshot()
        if method == 'state':
            return self.state()
        if method == 'ports':
            self.refresh_ports()
            return {'ports': list(self._ports)}
        if method == 'connect':
            return self.connect(params.get('port', ''))
        if method == 'disconnect':
            self.disconnect()
            return self.state()
        if method in JOB_MODES:
            return self.start_job(method, params.get('dry_run', False))
        if method == 'cancel':
            return self.cancel_job()
        raise ValueError('Unknown method')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='', help='自动连接的串口；留空则选择第一个 USB 串口')
    args = parser.parse_args()
    session = EncoderSession(args.port)
    try:
        for line in sys.stdin:
            request = {}
            try:
                request = json.loads(line)
                result = session.dispatch(request['method'], request.get('params', {}))
                response = {'id': request.get('id'), 'result': result}
            except Exception as exc:  # noqa: BLE001 - 边界必须把错误送回扩展
                response = {'id': request.get('id'), 'error': str(exc)}
            print(json.dumps(response, ensure_ascii=False, allow_nan=False), flush=True)
    finally:
        session.dt7.disconnect()
        session.disconnect()


if __name__ == '__main__':
    main()
