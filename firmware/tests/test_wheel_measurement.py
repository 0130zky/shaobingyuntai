#!/usr/bin/env python3
"""Independent wheel minima, time-based recovery and invalid-window regressions."""
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from wheel_measurement import measure_shot

class MeasurementTest(unittest.TestCase):
    def wave(self):
        t=np.arange(0,2,.002)
        d=np.zeros((len(t),25));d[:,3]=5750;d[:,4]=16.45;d[:,21]=1
        d[:,5]=(t>=1.1).astype(int)
        # Different baselines, minimum times, drop depths and recovery speeds.
        for w,(base,depth,onset,duration) in enumerate(((5750,1000,.98,.040),(5740,1300,.982,.070),(5760,1600,.984,.100))):
            d[:,w]=np.interp(t,[0,onset,onset+.006,onset+.006+duration,2],[base,base,base-depth,base,base])
        return t,d,int(np.searchsorted(t,1.1))
    def test_each_wheel_uses_own_minimum_and_baseline(self):
        t,d,i=self.wave();s=measure_shot(t,d,i,5750)
        self.assertTrue(s['valid'],s['note'])
        np.testing.assert_allclose(s['bases'],[5750,5740,5760],atol=.01)
        np.testing.assert_allclose(s['mins'],[4750,4440,4160],atol=.01)
        np.testing.assert_allclose(s['drop_rpm'],[1000,1300,1600],atol=.01)
        np.testing.assert_allclose(s['after_rpm'],[5750,5740,5760],atol=.01)
        self.assertLess(s['min_ms'][0],s['min_ms'][1])
        self.assertLess(s['min_ms'][1],s['min_ms'][2])
        self.assertLess(s['recovery_ms'][0],s['recovery_ms'][1])
        self.assertLess(s['recovery_ms'][1],s['recovery_ms'][2])
        self.assertLess(abs(s['seg']['t'][np.searchsorted(s['seg']['t'],0)]),.003)
    def test_truncated_recovery_is_not_reported_as_success(self):
        t,d,i=self.wave();d[t>1.06,0]=6000
        s=measure_shot(t,d,i,5750)
        self.assertTrue(np.isnan(s['recovery_ms'][0]))
        self.assertGreaterEqual(s['overshoot_rpm'][0],250)
        self.assertIn('未全部恢复',s['note'])
    def test_no_drop_off_target_change_and_gaps(self):
        t,d,i=self.wave();d[:,:3]=5750
        self.assertFalse(measure_shot(t,d,i,5750)['valid'])
        t,d,i=self.wave();d[:,0]=0
        self.assertFalse(measure_shot(t,d,i,5750)['valid'])
        t,d,i=self.wave();d[t>1.2,3]=5700
        self.assertIn('目标转速变化',measure_shot(t,d,i,5750)['note'])
        t,d,i=self.wave();t[600:]+=.1
        self.assertIn('断流',measure_shot(t,d,i,5750)['note'])

class EventTest(unittest.TestCase):
    def test_counter_reset_and_rollover(self):
        import shot_scope as scope
        from unittest.mock import patch
        t=np.arange(6.)
        d=np.zeros((6,25));d[:,5]=[10,0,1,65535,0,1]
        def fake(t,d,i,target):
            return dict(t=t[i],speed=16.45,dup=False,onset_t=t[i],note='')
        with patch.object(scope,'measure_shot',side_effect=fake):
            shots=scope.analyze_all(t,d,5750)
        self.assertEqual([s['t'] for s in shots],[2.,4.,5.])
    def test_equal_speed_separate_drops_are_preserved(self):
        from shot_scope import mark_duplicates
        shots=[dict(t=1.,onset_t=.9,speed=16.45,dup=False,note=''),
               dict(t=1.2,onset_t=1.1,speed=16.45,dup=False,note='')]
        mark_duplicates(shots)
        self.assertFalse(shots[1]['dup'])

if __name__=='__main__':unittest.main()
