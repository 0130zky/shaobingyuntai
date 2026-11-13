#!/usr/bin/env python3
"""Sentry USB status parsing and read-only session lifecycle. No board."""
import math
import pathlib
import subprocess
import sys
import time
import types
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'tools'))
from gimbal_encoder_vscode import EncoderSession, parse_status_line, raw_from_mrad

LINE = (b'SENTRY seq=7 result=0 cal=0 valid=1 mask=15 ready=1 rc=1 master=1 armed=0 fault=0 '
        b'fold_state=2 fold_raw=38800 fold_mrad=-2000 fold_goal=-1500 fold_age=12 '
        b'pitch_raw=36039 pitch_mrad=1000 pitch_goal=1200 pitch_age=0 '
        b'ends=1,2,3,4 end_raw=45628,38800,29762,36039 err=0,1 temp=40,41 s1=2 sticks=0,0 '
        b'z_mode=7 level_raw=34370 level_err=-100 s2=3 '
        b'fold_pitch_target=29763 open_pitch_target=34370')


class ParseTest(unittest.TestCase):
    def test_full_status_line(self):
        sample = parse_status_line(LINE)
        self.assertIsNotNone(sample)
        self.assertEqual(sample['seq'], 7)
        self.assertEqual(sample['fold']['raw'], 38800)
        self.assertAlmostEqual(sample['fold']['deg'], -114.591559, places=5)
        self.assertEqual(sample['fold']['goal_mrad'], -1500)
        self.assertEqual(sample['fold']['goal_raw'], round(raw_from_mrad(-1500)))
        self.assertEqual(sample['fold']['error_raw'],
                         sample['fold']['raw'] - sample['fold']['goal_raw'])
        self.assertEqual(sample['fold']['error_code'], 0)
        self.assertEqual(sample['pitch']['raw'], 36039)
        self.assertEqual(sample['pitch']['error_code'], 1)
        self.assertEqual(sample['pitch']['temp_c'], 41)
        self.assertEqual(sample['end_raw'], [45628, 38800, 29762, 36039])
        self.assertEqual(sample['sticks'], [0, 0])
        self.assertEqual(sample['z_mode'], 7)
        self.assertEqual(sample['open_pitch_target'], 34370)

    def test_raw_range_maps_to_mit_limits(self):
        self.assertAlmostEqual(raw_from_mrad(-12500), 0, places=6)
        self.assertAlmostEqual(raw_from_mrad(0), 32767.5, places=6)
        self.assertAlmostEqual(raw_from_mrad(12500), 65535, places=6)

    def test_never_seen_feedback_is_not_fresh(self):
        sample = parse_status_line(LINE.replace(b'fold_age=12', b'fold_age=4294967295'))
        self.assertFalse(sample['fold']['seen'])
        self.assertTrue(sample['pitch']['seen'])

    def test_corruption_and_unknown_lines_are_rejected(self):
        broken = [
            b'',
            b'STATUS seq=1',
            LINE.replace(b'pitch_raw=36039 ', b''),
            LINE.replace(b'fold_raw=38800', b'fold_raw=70000'),
            LINE.replace(b'fold_mrad=-2000', b'fold_mrad=nan'),
            LINE.replace(b'err=0,1', b'err=0'),
            LINE.replace(b'seq=7', b'seq=7 seq=8'),
            LINE.replace(b'fold_raw=38800', b'fold_raw=38800 fold_raw=1'),
        ]
        for line in broken:
            with self.subTest(line=line[:40]):
                self.assertIsNone(parse_status_line(line))

    def test_missing_optional_goal_still_reports_position(self):
        line = LINE.replace(b' fold_goal=-1500', b'').replace(b' pitch_goal=1200', b'')
        sample = parse_status_line(line)
        self.assertIsNotNone(sample)
        self.assertEqual(sample['fold']['raw'], 38800)
        self.assertIsNone(sample['fold']['goal_raw'])
        self.assertIsNone(sample['fold']['error_raw'])


class SessionTest(unittest.TestCase):
    def test_loopback_reads_without_writing_commands(self):
        session = EncoderSession()
        try:
            session.connect('loop://')
            state = session.state()
            self.assertTrue(state['connected'])
            self.assertEqual(state['port'], 'loop://')
            session._serial.write(LINE + b'\r\n')
            deadline = time.time() + 3
            while session.state()['received'] < 1 and time.time() < deadline:
                time.sleep(.02)
            state = session.state()
            self.assertGreaterEqual(state['received'], 1)
            self.assertEqual(state['invalid'], 0)
            self.assertEqual(state['latest']['fold']['raw'], 38800)
            self.assertGreaterEqual(len(state['history']), 1)
            self.assertLess(state['data_age_s'], 1)
        finally:
            session.disconnect()
        self.assertFalse(session.state()['connected'])

    def test_invalid_lines_are_counted_not_fatal(self):
        session = EncoderSession()
        try:
            session.connect('loop://')
            session._serial.write(b'noise\r\n' + LINE + b'\r\n')
            deadline = time.time() + 3
            while session.state()['received'] < 1 and time.time() < deadline:
                time.sleep(.02)
            state = session.state()
            self.assertEqual(state['received'], 1)
            self.assertGreaterEqual(state['invalid'], 1)
        finally:
            session.disconnect()

    def test_connect_without_port_explains_the_problem(self):
        session = EncoderSession()
        session._ports = []
        with self.assertRaises(ValueError):
            session.connect('')
        self.assertFalse(session.state()['connected'])


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'tools/sentry_build_flash.sh'
POSIX = unittest.skipIf(sys.platform == 'win32', '编译烧录脚本是 bash，Windows 上不执行')


class BuildScriptTest(unittest.TestCase):
    """只跑 --dry-run：不编译、不连接硬件。"""

    @POSIX
    def test_dry_run_shows_build_and_flash_commands(self):
        result = subprocess.run([str(SCRIPT), 'flash', '--dry-run'],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('cmake --preset Sentry', result.stdout)
        self.assertIn('target/stm32h7x.cfg', result.stdout)
        self.assertIn('build/SentryH7/sentry_dm_mc02.elf', result.stdout)
        self.assertIn('verify reset exit', result.stdout)

    @POSIX
    def test_build_only_never_invokes_openocd(self):
        result = subprocess.run([str(SCRIPT), 'build', '--dry-run'],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn('openocd', result.stdout)

    @POSIX
    def test_unknown_mode_is_a_usage_error(self):
        result = subprocess.run([str(SCRIPT), 'erase', '--dry-run'],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 1)
        self.assertIn('未知参数', result.stderr)


class BuildJobTest(unittest.TestCase):
    @POSIX
    def test_dry_run_job_reaches_done_and_keeps_log(self):
        session = EncoderSession()
        try:
            session.dispatch('flash', {'dry_run': True})
            deadline = time.time() + 20
            while session.state()['job']['running'] and time.time() < deadline:
                time.sleep(.05)
            job = session.state()['job']
            self.assertFalse(job['running'])
            self.assertEqual(job['exit_code'], 0)
            self.assertEqual(job['phase'], 'done')
            self.assertIsNone(job['error'])
            text = '\n'.join(row['text'] for row in job['output'])
            self.assertIn('[build]', text)
            self.assertIn('[flash]', text)
            self.assertEqual(session.state()['connected'], False)
        finally:
            session.disconnect()

    def test_second_job_is_refused_while_running(self):
        session = EncoderSession()
        try:
            session.job = types.SimpleNamespace(running=True)
            with self.assertRaises(ValueError):
                session.start_job('build')
        finally:
            session.job = None

    def test_cancel_without_running_job_explains(self):
        session = EncoderSession()
        with self.assertRaises(ValueError):
            session.dispatch('cancel', {})


if __name__ == '__main__':
    unittest.main()
