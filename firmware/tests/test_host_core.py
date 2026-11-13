"""Portable protocol, timing, transport and recorder lifecycle tests. No board."""
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
import unittest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))
from board_clock import BoardClock, reconstruct_times, collapse_times, signed_delta_ms
from process_support import FileLock, process_start
from serial_transport import SerialPort
from shot_monitor import identity
from shot_scope_vscode import Dashboard
from telemetry_protocol import Decoder, TAIL, DIAG_TAIL


class HostCoreTest(unittest.TestCase):
    def test_board_wrap_duplicate_and_reset(self):
        clock = BoardClock()
        self.assertEqual(clock.advance(65530), 0)
        self.assertAlmostEqual(clock.advance(2), .008)
        self.assertEqual(clock.advance(2), .008)
        self.assertFalse(clock.changed)
        clock.advance(1000)
        clock.advance(1)
        self.assertTrue(clock.reset)
        self.assertEqual(clock.epoch, 1)
        self.assertEqual(reconstruct_times([10, 10, 13]), [0, 0, .003])
        self.assertEqual(collapse_times([10, 10, 13]), ([0, .003], [1, 2]))
        self.assertEqual(signed_delta_ms(100, 102), -2)
        self.assertEqual(signed_delta_ms(2, 65530), 8)

    def test_protocol_has_no_posix_dependency(self):
        code = ('import sys;sys.path.insert(0,sys.argv[1]);'
                'sys.modules["fcntl"]=None;sys.modules["termios"]=None;'
                'from telemetry_protocol import Decoder;assert Decoder().feed(b"noise")==[]')
        subprocess.run([sys.executable, '-c', code, str(TOOLS)], check=True)

    def test_diagnostic_completion_time_can_follow_sample_capture(self):
        sample = [0.] * 25
        sample[7] = 100
        for completion, accepted in ((102, True), (120, True), (121, False), (65435, False)):
            diagnostic = [0.] * 24
            diagnostic[0] = 1
            diagnostic[1] = completion
            wire = struct.pack('<24f', *diagnostic) + DIAG_TAIL + struct.pack('<25f', *sample) + TAIL
            record = Decoder().feed_records(wire)[0]
            self.assertEqual(record[1] is not None, accepted)

    def test_portable_lock_and_process_identity(self):
        token = process_start(os.getpid())
        self.assertIsNotNone(token)
        self.assertEqual(token, process_start(os.getpid()))
        self.assertIsNone(process_start(-1))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'lock'
            with FileLock(path):
                with self.assertRaises(BlockingIOError):
                    with FileLock(path):
                        self.fail('duplicate owner')
            with FileLock(path):
                pass

    def test_loopback_transport(self):
        port = SerialPort('loop://', 460800)
        try:
            data = struct.pack('<4f', 1, 2, 3, 4)
            self.assertEqual(port.write(data), len(data))
            self.assertEqual(port.read(timeout=.2), data)
            self.assertEqual(port.read(timeout=.01), b'')
        finally:
            port.close()

    def test_dashboard_does_not_invent_samples_for_a_frozen_board(self):
        with tempfile.TemporaryDirectory() as tmp:
            dashboard = Dashboard(tmp)
            row = [0.] * 25
            row[7] = 10
            dashboard.append(row)
            for _ in range(20):
                dashboard.append(row)
            self.assertEqual(len(dashboard.samples), 1)
            self.assertEqual(dashboard.clock, 0)
            row = row.copy(); row[7] = 15
            dashboard.append(row)
            self.assertEqual(dashboard.clock, .005)
            dashboard.reset()
            row[7] = 0
            dashboard.append(row)
            self.assertEqual(dashboard.clock, 0)

    def test_hidden_recorder_start_duplicate_and_graceful_stop(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            env = dict(os.environ, PYTHONUTF8='1')

            def cli(command):
                args = [sys.executable, '-X', 'utf8', str(TOOLS / 'shot_monitor.py'), command,
                        '--directory', str(directory)]
                if command == 'start':
                    args += ['--port', 'loop://']
                return subprocess.run(args, text=True, encoding='utf-8', capture_output=True,
                                      env=env, timeout=10, check=True)

            value = {}
            try:
                cli('start')
                value = json.loads((directory / 'status.json').read_text(encoding='utf-8'))
                self.assertTrue(value['connected'])
                self.assertTrue(identity(directory))
                self.assertIn('Already running', cli('start').stdout)
                cli('stop')
                self.assertFalse(identity(directory))
                status = json.loads((directory / 'status.json').read_text(encoding='utf-8'))
                self.assertFalse(status['running'])
                events = Path(value['events_file']).read_text(encoding='utf-8')
                self.assertIn('monitor_stopped', events)
                self.assertNotIn('monitor_error', events)
            finally:
                # A session-scoped request also cleans up if a CLI assertion fails.
                if value.get('session_id'):
                    (directory / 'stop_request.json').write_text(
                        json.dumps({'session_id': value['session_id']}), encoding='utf-8')
                    for _ in range(50):
                        if not identity(directory):
                            break
                        time.sleep(.1)


if __name__ == '__main__':
    unittest.main()
