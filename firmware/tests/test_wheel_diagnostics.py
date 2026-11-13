#!/usr/bin/env python3
"""Mixed UART frames, diagnostic expiry, replay and missing-data regressions."""
import csv
from pathlib import Path
import struct
import sys
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from telemetry_io import Decoder, TAIL, DIAG_TAIL, FIELDS, DIAG_FIELDS, RECORD_FIELDS
from shot_scope_vscode import Dashboard


def sample(ms):
    return [5782.] * 4 + [16.45, 1., 0., float(ms), 0., 4.] + [0.] * 11 + [1., 0., 0., 1.]


def diagnostic(ms):
    return [1., float(ms), 3200., 4100., 80., 120., 1., 2., 1., 1., 0.,
            10., -5., -5., 5792., 5777., 5777., 1., 1., 0., 0., 4., 6., 9.]


def packet(values, tail):
    return struct.pack('<%df' % len(values), *values) + tail


class DiagnosticTest(unittest.TestCase):
    def test_mixed_partial_stream_keeps_wire_order(self):
        wire = (b'junk' + packet(sample(100), TAIL) + packet(diagnostic(101), DIAG_TAIL)
                + packet(sample(104), TAIL) + packet(diagnostic(105), DIAG_TAIL)
                + packet(sample(107), TAIL))
        decoder = Decoder(); result = []
        for i in range(0, len(wire), 13):
            result.extend(decoder.feed_records(wire[i:i+13]))
        self.assertEqual(len(result), 3)
        self.assertIsNone(result[0][1])
        self.assertEqual(result[1][1][1], 101)
        self.assertEqual(result[2][1][1], 105)
        self.assertEqual(decoder.invalid, 0)
        self.assertEqual(len(Decoder().feed(wire)[0]), 25)

    def test_expiry_wrap_reset_and_invalid_diagnostic(self):
        decoder = Decoder()
        decoder.feed_records(packet(diagnostic(65530), DIAG_TAIL))
        self.assertIsNotNone(decoder.feed_records(packet(sample(4), TAIL))[0][1])
        self.assertIsNone(decoder.feed_records(packet(sample(250), TAIL))[0][1])
        decoder.feed_records(packet(diagnostic(1000), DIAG_TAIL) + packet(sample(1001), TAIL))
        self.assertIsNone(decoder.feed_records(packet(sample(3), TAIL))[0][1])
        bad = diagnostic(4); bad[18] = 100  # impossible: more limit cycles than window cycles
        decoder.feed_records(packet(bad, DIAG_TAIL))
        self.assertEqual(decoder.invalid, 1)

    def test_invalid_shot_without_origin_does_not_break_dashboard(self):
        with tempfile.TemporaryDirectory() as temp:
            app = Dashboard(temp)
            for ms in range(0, 2100, 2):
                row = sample(ms); row[5] = float(ms >= 1000)
                app.append(row + diagnostic(ms) + [0.])
            app.finish()
            self.assertEqual(len(app.shots), 1)
            self.assertFalse(app.shots[0]['valid'])
            self.assertNotIn('diagnostic_seg', app.shots[0])

    def test_old_and_new_csv_replay(self):
        with tempfile.TemporaryDirectory() as temp:
            app = Dashboard(temp)
            for extended in (False, True):
                path = Path(temp) / ('new.csv' if extended else 'old.csv')
                with path.open('w', newline='') as stream:
                    writer = csv.writer(stream)
                    writer.writerow(['host_unix_s', 'host_elapsed_s', *(RECORD_FIELDS if extended else FIELDS)])
                    for ms in range(100, 110):
                        writer.writerow([1, ms / 1000, *sample(ms), *(diagnostic(100) + [ms-100] if extended else [])])
                state = app.replay(path)
                self.assertEqual(state['latest']['rpm_12h'], 5782)
                self.assertEqual(state['latest']['pid_period_us'], 3200 if extended else None)
                self.assertEqual(state['diagnostic_live'][-1][1], 1 if extended else None)
            app.reset()


if __name__ == '__main__':
    unittest.main()
