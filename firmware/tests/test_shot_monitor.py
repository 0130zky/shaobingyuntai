#!/usr/bin/env python3
"""Protocol/event tests plus detached recording/reconnect tests on a pseudo UART."""
import csv
import json
import os
from pathlib import Path
if os.name != 'nt':
    import pty
import select
import struct
import subprocess
import sys
import tempfile
import time
import unittest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))
from telemetry_io import Decoder, TAIL, DIAG_TAIL
from shot_monitor import Analysis


def values(count=5, speed=16.45, sample=100):
    return [5750., 5750., 5750., 5750., speed, float(count), 50., float(sample),
            10., 4., 100., 100., 100., 100., 100., 100., 0., 0., 0., .01, .01,
            1., 0., 0., 1.]


class MonitorTest(unittest.TestCase):
    def test_decoder_resynchronizes_partial_and_corrupt_frames(self):
        good = struct.pack('<25f', *values()) + TAIL
        corrupt = values()
        corrupt[24] = 999
        wire = b'junk' + good[40:] + struct.pack('<25f', *corrupt) + TAIL + good * 3
        decoder = Decoder()
        frames = []
        for i in range(0, len(wire), 17):
            frames.extend(decoder.feed(wire[i:i+17]))
        self.assertEqual(len(frames), 3)
        self.assertGreater(decoder.discarded, 0)

    def test_events_counter_wrap_freeze_jam_and_reset(self):
        from telemetry_io import FIELDS
        events = []
        analysis = Analysis(lambda event, **kw: events.append((event, kw)), 16.45, .05)
        for t, count, sample, jam, speed in [(0, 65535, 65500, 0, 16.45),
                                           (.05, 0, 14, 1, 16.46),
                                           (.2, 0, 164, 0, 16.46),
                                           (.3, 1, 264, 0, 16.9),
                                           (.4, 0, 364, 0, 0),
                                           (.5, 0, 2, 0, 0)]:
            v = values(count, speed, sample)
            v[22] = jam
            analysis.sample(dict(zip(FIELDS, v)), t)
        shots = [data for event, data in events if event == 'shot_message']
        self.assertEqual(len(shots), 2)
        self.assertTrue(shots[0]['in_target_band'])
        self.assertFalse(shots[1]['in_target_band'])
        names = [event for event, _ in events]
        self.assertIn('dial_jam', names)
        self.assertIn('dial_jam_cleared', names)
        self.assertIn('wheel_feedback_frozen_suspected', names)
        self.assertIn('shot_counter_reset', names)
        self.assertIn('board_clock_discontinuity', names)

    @unittest.skipIf(os.name == 'nt', 'POSIX PTY test; portable loopback lifecycle is tested separately')
    def test_background_rotation_reconnect_stop_and_exclusion(self):
        with tempfile.TemporaryDirectory(prefix='shot-monitor-test-') as temp:
            base = Path(temp)
            record = base / 'records'
            master, slave = pty.openpty()
            alias = base / 'serial'
            alias.symlink_to(os.ttyname(slave))
            command = [sys.executable, str(TOOLS / 'shot_monitor.py')]

            def cli(action, *extra):
                return subprocess.run(command + [action, '--directory', str(record), *extra],
                                      text=True, capture_output=True, timeout=12, check=True)

            def status():
                try:
                    return json.loads((record / 'status.json').read_text())
                except (OSError, ValueError):
                    return {}

            def wait(predicate, seconds=6):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    if predicate():
                        return
                    time.sleep(.05)
                self.fail('Condition timed out; status=%r' % status())

            sequence = 0

            def pump(fd, count, speed, n=80):
                nonlocal sequence
                for _ in range(n):
                    sequence += 1
                    v = values(count, speed, (65000 + sequence*10) & 65535)
                    v[13] = sequence % 300
                    if sequence % 10 == 0:
                        diagnostic = [1., v[7], 3200., 4100., 80., 100., 1., 1., 1., 0., 32.,
                                      0., 0., 0., 5750., 5750., 5750., 0., 0., 0., 0., 0., 6., 9.]
                        os.write(fd, struct.pack('<24f', *diagnostic) + DIAG_TAIL)
                    os.write(fd, struct.pack('<25f', *v) + TAIL)
                    time.sleep(.01)

            try:
                cli('start', '--port', str(alias), '--baudrate', '115200',
                    '--rotate-minutes', '.005', '--label', 'integration-test')
                wait(lambda: status().get('connected'))
                pid = status()['pid']
                self.assertIn('Already running', cli('start', '--port', str(alias)).stdout)
                self.assertEqual(status()['pid'], pid)
                from telemetry_io import SerialPort
                with self.assertRaises(OSError):
                    opened = SerialPort(str(alias), 115200)
                    opened.close()
                # Explicit calibration mailbox is the only transmit path.
                (record/'dial_phase_command.json').write_text(json.dumps({
                    'phase_centideg':-200,'created_unix_s':time.time()}))
                self.assertTrue(select.select([master],[],[],3)[0])
                packet=os.read(master,100)
                expected=bytearray(b'\xa6\x5a\x01')+struct.pack('<h',-200)
                checksum=0
                for byte in expected:checksum^=byte
                expected.append(checksum)
                self.assertEqual(packet,expected)
                ack=bytearray(expected);ack[2]=0x81;ack[5]^=0x80
                os.write(master,ack[:3]);time.sleep(.03);os.write(master,ack[3:])
                wait(lambda: status().get('dial_phase_reply',{}).get('accepted'))
                self.assertEqual(status()['dial_phase_reply']['phase_deg'],-2)
                pump(master, 5, 16.45)
                pump(master, 6, 16.46)
                pump(master, 7, 16.9)
                wait(lambda: status().get('messages') == 2)
                session = Path(status()['session_directory'])
                self.assertGreaterEqual(len(list(session.glob('telemetry_*.csv'))), 2)
                os.close(master)
                master = None
                os.close(slave)
                slave = None
                wait(lambda: not status().get('connected'))
                master, slave = pty.openpty()
                replacement = base / 'serial-new'
                replacement.symlink_to(os.ttyname(slave))
                replacement.replace(alias)
                wait(lambda: status().get('connected'))
                pump(master, 8, 16.44, n=150)
                wait(lambda: status().get('latest', {}).get('shot_count_u16') == 8)
                self.assertIn('serial_disconnected', cli('events', '--lines', '100').stdout)
                self.assertIn('Stopped', cli('stop').stdout)
                self.assertFalse(status()['running'])
                self.assertFalse(json.loads(cli('status').stdout)['running'])
                events = [json.loads(line) for line in (session / 'events.jsonl').read_text().splitlines()]
                shots = [event for event in events if event['event'] == 'shot_message']
                self.assertEqual(len(shots), 2)
                self.assertTrue(shots[0]['in_target_band'])
                self.assertFalse(shots[1]['in_target_band'])
                rows = []
                for path in session.glob('telemetry_*.csv'):
                    with path.open() as stream:
                        rows.extend(csv.DictReader(stream))
                self.assertGreater(len(rows), 200)
                self.assertIn('host_unix_s', rows[0])
                self.assertTrue(any(row['pid_period_us'] == '3200.0' for row in rows))
                self.assertTrue(any(row['diag_age_ms'] for row in rows))
            finally:
                cli('stop')
                if master is not None:
                    os.close(master)
                if slave is not None:
                    os.close(slave)


if __name__ == '__main__':
    unittest.main()
