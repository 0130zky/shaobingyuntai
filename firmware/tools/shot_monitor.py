#!/usr/bin/env python3
"""Persistent telemetry recorder with an explicit dial-calibration command mailbox. POSIX stdlib transport; Windows uses pyserial."""
import argparse
from collections import deque
import csv
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import struct
import sys
import time
import uuid
from zoneinfo import ZoneInfo

from telemetry_protocol import AuxiliaryDecoder, Decoder, FIELDS, DIAG_FIELDS, RECORD_FIELDS
from serial_transport import SerialPort, available_ports
from process_support import FileLock, process_start, detached_options
from board_clock import BoardClock, signed_delta_ms

HERE = Path(__file__).resolve().parent
DEFAULT_DIRECTORY = HERE / 'records' / 'monitor'
TZ = ZoneInfo('Asia/Shanghai')


def timestamp():
    return dt.datetime.now(TZ).isoformat(timespec='milliseconds')


def load_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def save_json(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    temp.replace(path)


def identity(directory):
    value = load_json(directory / 'pid.json')
    if value.get('process_start') and value['process_start'] == process_start(value.get('pid')):
        return value
    return {}


def find_port():
    ports = available_ports()
    if len(ports) != 1:
        raise ValueError('Specify --port; found ports: %s' % ports)
    return ports[0]


class Analysis:
    """Event hints only; the recorder never controls the board."""
    def __init__(self, emit, target, tolerance):
        self.emit = emit
        self.target = target
        self.tolerance = tolerance
        self.messages = 0
        self.shot_count = None
        self.last_board = None
        self.last_host = None
        self.flags = {}
        self.frozen_tuple = None
        self.frozen_since = None
        self.stall_since = None

    def reconnect(self):
        self.shot_count = self.last_board = self.last_host = None
        self.frozen_tuple = self.frozen_since = self.stall_since = None

    def flag(self, name, active, frame):
        if self.flags.get(name, False) != active:
            self.emit(name if active else name + '_cleared', frame=frame)
        self.flags[name] = active

    def sample(self, frame, elapsed):
        board = int(frame['sample_ms_u16'])
        reset = False
        if self.last_board is not None:
            board_delta = (board - self.last_board) & 65535
            host_delta = (elapsed - self.last_host) * 1000
            if abs(board_delta - host_delta) > max(1500, host_delta + 500):
                reset = True
                self.emit('board_clock_discontinuity', frame=frame)
        self.last_board, self.last_host = board, elapsed
        count = int(frame['shot_count_u16'])
        if reset:
            self.shot_count = None
        if self.shot_count is None:
            self.emit('shot_counter_baseline', frame=frame)
        elif count != self.shot_count:
            delta = (count - self.shot_count) & 65535
            if delta > 32768:
                self.emit('shot_counter_reset', frame=frame)
            else:
                self.messages += delta
                self.emit('shot_message', delta=delta,
                          in_target_band=abs(frame['bullet_speed_mps'] - self.target) <= self.tolerance,
                          note='Received message count; duplicates/missing events require waveform review',
                          frame=frame)
        self.shot_count = count
        self.flag('dial_jam', bool(frame['dial_jammed']), frame)
        self.flag('dial_feedback_invalid', not bool(frame['dial_fb_ok']), frame)
        self.flag('referee_stale', frame['referee_age_ms'] > 300, frame)
        self.flag('wheel_command_near_limit',
                  any(abs(frame[name]) >= .97 * 16384 for name in FIELDS[10:13]), frame)
        frozen = tuple(frame[name] for name in FIELDS[:3] + FIELDS[13:16])
        if frozen != self.frozen_tuple:
            self.frozen_tuple, self.frozen_since = frozen, elapsed
        suspected = (frame['target_rpm'] != 0 and self.frozen_since is not None
                     and elapsed - self.frozen_since >= .1)
        self.flag('wheel_feedback_frozen_suspected', suspected, frame)
        stalled = (frame['dial_fb_ok'] and abs(frame['dial_speed_dps']) < 5
                   and abs(frame['dial_output_norm']) > .15)
        if not stalled:
            self.stall_since = None
        elif self.stall_since is None:
            self.stall_since = elapsed
        self.flag('dial_stall_suspected', self.stall_since is not None
                  and elapsed - self.stall_since >= .5, frame)


def run(args):
    directory = args.directory.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    with FileLock(directory / 'monitor.lock'):
        session_id = args.session_id or uuid.uuid4().hex
        session = directory / (dt.datetime.now(TZ).strftime('%Y%m%d_%H%M%S') + '_' + session_id[:8])
        session.mkdir()
        pid = {'pid': os.getpid(), 'session_id': session_id, 'session_directory': str(session), 'process_start': process_start(os.getpid())}
        save_json(directory / 'pid.json', pid)
        status = dict(pid, running=True, connected=False, port=args.port,
                      baudrate=args.baudrate, channels=25, diagnostic_channels=24, frames=0, target_mps=args.target,
                      tolerance_mps=args.tolerance, events_file=str(session / 'events.jsonl'),
                      started_at=timestamp(), label=args.label, timezone='Asia/Shanghai')
        build = HERE.parent / 'build' / 'Shooter' / 'robot_template.elf'
        if build.exists():
            status['local_build_sha256'] = hashlib.sha256(build.read_bytes()).hexdigest()
        save_json(session / 'session.json', status)
        stop = False

        def request_stop(signum, frame):
            nonlocal stop
            stop = True

        signal.signal(signal.SIGTERM, request_stop)
        signal.signal(signal.SIGINT, request_stop)
        command_path = directory / 'dial_phase_command.json'
        command_path.unlink(missing_ok=True)  # stale requests must not run after restart
        start = time.monotonic()
        serial = None
        output = None
        writer = None
        phase_rx = AuxiliaryDecoder()
        pitch_feedback_state = None
        decoder = Decoder()
        board_clock = BoardClock()
        segment_start = 0
        segment = 0
        next_reconnect = 0
        last_valid = start
        last_save = 0
        last_error = None
        with (session / 'events.jsonl').open('a', buffering=1, encoding='utf-8') as event_file:
            def emit(event, **details):
                event_file.write(json.dumps(dict(time=timestamp(),
                    elapsed_s=round(time.monotonic() - start, 6), event=event, **details),
                    ensure_ascii=False, allow_nan=False) + '\n')
                status['last_event'] = event

            analysis = Analysis(emit, args.target, args.tolerance)
            emit('monitor_started', port=args.port, label=args.label)
            print('MONITOR STARTED', session, flush=True)
            try:
                while not stop:
                    if load_json(directory / 'stop_request.json').get('session_id') == session_id:
                        break
                    now = time.monotonic()
                    if serial is None and now >= next_reconnect:
                        try:
                            serial = SerialPort(args.port, args.baudrate)
                            phase_rx = AuxiliaryDecoder()
                            pitch_feedback_state = None
                            decoder = Decoder()
                            board_clock = BoardClock()
                            analysis.reconnect()
                            last_valid = now
                            status.update(connected=True, error=None)
                            last_error = None
                            emit('serial_connected', port=args.port)
                        except (OSError, ValueError) as exc:
                            message = str(exc)
                            if message != last_error:
                                emit('serial_open_failed', error=message)
                                last_error = message
                            status.update(connected=False, error=message)
                            next_reconnect = now + 1
                    if serial is not None:
                        try:
                            if command_path.exists():
                                try:
                                    command = json.loads(command_path.read_text(encoding="utf-8"))
                                    command_path.unlink(missing_ok=True)
                                    if not 0 <= time.time()-command['created_unix_s'] <= 3:
                                        raise ValueError('Expired calibration request')
                                    phase = command.get('phase_centideg',0)
                                    endpoint=command.get('pitch_endpoint')
                                    if endpoint is not None:
                                        if type(endpoint) is not int or endpoint not in (1,2):raise ValueError('Invalid endpoint')
                                        packet=bytearray(b'\xa6\x5a\x03')+struct.pack('<h',endpoint)
                                    else:
                                        if type(phase) is not int or abs(phase)>1000:raise ValueError('Invalid phase')
                                        packet=bytearray(b'\xa6\x5a\x01')+struct.pack('<h',phase)
                                    checksum = 0
                                    for byte in packet:
                                        checksum ^= byte
                                    packet.append(checksum)
                                    sent = serial.write(packet)
                                    if sent != len(packet):
                                        raise OSError('Incomplete calibration command; not retried')
                                    emit('dial_phase_command_sent', phase_deg=phase/100,
                                         note='Await board reply; board applies only while master disabled')
                                except (ValueError, KeyError, TypeError) as exc:
                                    emit('dial_phase_command_rejected', error=str(exc))
                            chunk = serial.read(timeout=.2)
                            if chunk:
                                for kind,value in phase_rx.feed(chunk):
                                    if kind == 0xc1:
                                        previous = status.get('pitch_encoder')
                                        snapshot = dict(value, received_unix_s=time.time())
                                        status['pitch_encoder'] = snapshot
                                        fresh = snapshot['feedback_age_ms'] <= 100
                                        if pitch_feedback_state is None:
                                            emit('pitch_feedback_initial', fresh=fresh, encoder=snapshot)
                                        elif fresh != pitch_feedback_state:
                                            event = 'pitch_feedback_recovered' if fresh else 'pitch_feedback_lost'
                                            emit(event, encoder=snapshot, previous_encoder=previous)
                                        elif fresh and previous and snapshot['error_code'] != previous['error_code']:
                                            emit('pitch_drive_state_changed', encoder=snapshot,
                                                 previous_error_code=previous['error_code'])
                                        pitch_feedback_state = fresh
                                    elif kind == 0xc2:
                                        code, detail = value
                                        reason = {
                                            1: 'switch_off', 2: 'rc_lost', 3: 'feedback_timeout',
                                            4: 'angle_limit', 5: 'drive_fault', 6: 'drive_disabled',
                                            7: 'stick_not_centered', 8: 'can_tx_timeout',
                                        }.get(code, 'unknown')
                                        status['pitch_exit'] = dict(code=code, reason=reason, detail=detail,
                                                                    received_unix_s=time.time())
                                        emit('pitch_exit', **status['pitch_exit'])
                                    elif kind in (0xb1,0xb2,0xb3,0xb4,0xbe,0xbf):
                                        status['pitch_limit_reply']=dict(code=kind,value_mrad=value,received_unix_s=time.time())
                                        emit('pitch_limit_board_reply',**status['pitch_limit_reply'])
                                    else:
                                        status['dial_phase_reply']=dict(phase_deg=value/100,accepted=kind==0x81,received_unix_s=time.time())
                                        emit('dial_phase_board_reply',**status['dial_phase_reply'])
                                frames = decoder.feed_records(chunk)
                                for values, diagnostic in frames:
                                    now = time.monotonic()
                                    elapsed = now - start
                                    if output is None or now - segment_start >= args.rotate_minutes * 60:
                                        if output is not None:
                                            output.close()
                                        segment += 1
                                        path = session / ('telemetry_%04d.csv' % segment)
                                        output = path.open('w', newline='')
                                        writer = csv.writer(output)
                                        writer.writerow(['host_unix_s', 'host_elapsed_s', *RECORD_FIELDS])
                                        segment_start = now
                                        status['csv_file'] = str(path)
                                        emit('csv_segment_started', path=str(path))
                                    frame = dict(zip(FIELDS, values))
                                    if diagnostic is not None:
                                        frame.update(zip(DIAG_FIELDS, diagnostic))
                                        frame['diag_age_ms'] = max(0, signed_delta_ms(values[7], diagnostic[1]))
                                    writer.writerow([time.time(), elapsed, *[frame.get(key) for key in RECORD_FIELDS]])
                                    analysis.sample(frame, elapsed)
                                    status['frames'] += 1
                                    status['latest'] = frame
                                    board_clock.advance(values[7])
                                    status['last_packet_at'] = timestamp()
                                    if board_clock.changed:
                                        status['last_sample_at'] = timestamp()
                                        last_valid = now
                            if time.monotonic() - last_valid > 5:
                                raise OSError('No valid telemetry for 5 seconds; reopening port')
                        except (OSError, ValueError) as exc:
                            serial.close()
                            serial = None
                            status.update(connected=False, error=str(exc))
                            emit('serial_disconnected', error=str(exc))
                            next_reconnect = time.monotonic() + 1
                    else:
                        time.sleep(.2)
                    now = time.monotonic()
                    if now - last_save >= 1:
                        if output is not None:
                            output.flush()
                        status.update(updated_at=timestamp(), messages=analysis.messages,
                                      active_flags=[k for k, v in analysis.flags.items() if v],
                                      last_valid_age_s=round(now - last_valid, 3),
                                      decoder_invalid=decoder.invalid, decoder_discarded_bytes=decoder.discarded)
                        save_json(directory / 'status.json', status)
                        last_save = now
            except BaseException as exc:
                status['error'] = repr(exc)
                emit('monitor_error', error=repr(exc))
                raise
            finally:
                if serial is not None:
                    serial.close()
                if output is not None:
                    output.close()
                emit('monitor_stopped')
                status.update(running=False, connected=False, updated_at=timestamp(),
                              messages=analysis.messages)
                save_json(directory / 'status.json', status)
                (directory / 'pid.json').unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ['start', 'run', 'status', 'stop', 'events']:
        p = commands.add_parser(name)
        p.add_argument('--directory', type=Path, default=DEFAULT_DIRECTORY)
        if name in ('start', 'run'):
            p.add_argument('--port')
            p.add_argument('--baudrate', type=int, choices=[115200, 460800], default=460800)
            p.add_argument('--rotate-minutes', type=float, default=30)
            p.add_argument('--target', type=float, default=16.45)
            p.add_argument('--tolerance', type=float, default=.05)
            p.add_argument('--label', default='')
            p.add_argument('--session-id', default='', help=argparse.SUPPRESS)
        if name == 'events':
            p.add_argument('--lines', type=int, default=20)
    args = parser.parse_args()
    directory = args.directory.resolve()
    if args.command in ('start', 'run'):
        import math
        if not all(math.isfinite(x) and x > 0 for x in
                   (args.rotate_minutes, args.target, args.tolerance)):
            parser.error('rotation, target and tolerance must be positive and finite')
        if args.command == 'start' and identity(directory):
            print('Already running:', json.dumps(identity(directory), ensure_ascii=False))
            return
        args.port = args.port or find_port()
        if args.command == 'run':
            run(args)
            return
        directory.mkdir(parents=True, exist_ok=True)
        token = uuid.uuid4().hex
        cmd = [sys.executable, '-X', 'utf8', '-u', str(Path(__file__).resolve()), 'run',
               '--directory', str(directory), '--port', args.port,
               '--baudrate', str(args.baudrate), '--rotate-minutes', str(args.rotate_minutes),
               '--target', str(args.target), '--tolerance', str(args.tolerance),
               '--label', args.label, '--session-id', token]
        with (directory / 'daemon.log').open('a') as log:
            child = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                     close_fds=True, **detached_options())
        for _ in range(40):
            value = load_json(directory / 'status.json')
            if value.get('session_id') == token:
                print(json.dumps(value, ensure_ascii=False, indent=2))
                return
            if child.poll() is not None:
                raise SystemExit('Recorder failed; read %s' % (directory / 'daemon.log'))
            time.sleep(.1)
        print('Recorder starting, PID', child.pid, '; see', directory / 'daemon.log')
    elif args.command == 'status':
        value = load_json(directory / 'status.json')
        value['running'] = bool(identity(directory))
        print(json.dumps(value, ensure_ascii=False, indent=2))
    elif args.command == 'stop':
        value = identity(directory)
        if not value:
            print('Recorder is not running')
            return
        save_json(directory / 'stop_request.json', {'session_id': value['session_id']})
        for _ in range(50):
            if not identity(directory):
                print('Stopped; records preserved in', value['session_directory'])
                return
            time.sleep(.1)
        raise SystemExit('Still stopping; check status before opening the serial port')
    else:
        value = load_json(directory / 'status.json')
        path = value.get('events_file')
        if not path or not Path(path).exists():
            raise SystemExit('No events recorded yet')
        if args.lines <= 0:
            parser.error('--lines must be positive')
        with Path(path).open() as stream:
            for line in deque(stream, maxlen=args.lines):
                print(line.rstrip())


if __name__ == '__main__':
    main()
