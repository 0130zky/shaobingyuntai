#!/usr/bin/env python3
"""Regression checks for offline wheel evaluation selection and recovery."""
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from evaluate_friction_wheels import evaluate_window, recovery_time


class FrictionEvaluationTest(unittest.TestCase):
    def wave(self):
        t = np.arange(0, 2, .002)
        data = np.zeros((len(t), 25))
        data[:, 3] = 5782
        for w, duration in enumerate((.04, .07, .10)):
            data[:, w] = np.interp(t, [0, .98, .986, .986 + duration, 2],
                                  [5782, 5782, 4182, 5782, 5782])
        return t, data, int(np.searchsorted(t, 1.1))

    def test_independent_recovery_and_same_sample_spread(self):
        t, d, i = self.wave()
        result = evaluate_window(t, d, i)
        self.assertTrue(result['usable'])
        np.testing.assert_allclose(result['drop_rpm'], [1600] * 3)
        self.assertLess(result['recovery_ms'][0], result['recovery_ms'][1])
        self.assertLess(result['recovery_ms'][1], result['recovery_ms'][2])
        # Equal independent troughs do not imply equal speeds during recovery.
        self.assertAlmostEqual(result['common_min_spread_rpm'], 0)
        self.assertGreater(result['peak_loaded_spread_rpm'], 500)

    def test_bad_selection_and_target_change(self):
        t, d, i = self.wave()
        d[:, :3] = 5782
        self.assertFalse(evaluate_window(t, d, i)['usable'])
        t, d, i = self.wave()
        d[t > 1.02, 3] = 6000
        self.assertIn('调目标', evaluate_window(t, d, i)['reason'])
        t, d, i = self.wave()
        t[600:] += .03
        self.assertIn('断流', evaluate_window(t, d, i)['reason'])

    def test_ripple_is_measured_not_filtered_out(self):
        t, d, i = self.wave()
        pre = t < .95
        d[pre, 0] += 60 * np.sin(t[pre] * 2 * np.pi * 30)
        result = evaluate_window(t, d, i)
        self.assertTrue(result['usable'])
        self.assertGreater(result['baseline_ripple_rpm'][0], 100)

    def test_full_real_time_hold_required(self):
        t = np.array([0., .01, .02, .03])
        self.assertTrue(np.isnan(recovery_time(t, np.full(4, 5782), 0, 5782)))
        t = np.arange(0, .12, .01)
        rpm = np.full(len(t), 5782.)
        rpm[3] = 6000
        self.assertAlmostEqual(recovery_time(t, rpm, 0, 5782), 40)


if __name__ == '__main__':
    unittest.main()
