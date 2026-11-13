#!/usr/bin/env python3
"""VS Code backend replay, incremental CSV, export and calibration safeguards."""
import csv
import datetime as dt
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from shot_scope_vscode import Dashboard
from telemetry_io import FIELDS

class DashboardTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.path=Path(self.temp.name)
        self.csv=self.path/'telemetry.csv'
        with self.csv.open('w',newline='') as stream:
            w=csv.writer(stream);w.writerow(['host_unix_s','host_elapsed_s',*FIELDS])
            for ms in range(0,2000,2):
                row=np.zeros(25);row[3]=5750;row[4]=16.45;row[5]=int(ms>=1100);row[7]=ms;row[21]=1
                for wheel,base in enumerate((5750,5740,5760)):
                    row[wheel]=np.interp(ms,[0,980,990,1050,2000],[base,base,base-1000-wheel*200,base,base])
                    row[10+wheel]=100+wheel;row[13+wheel]=200+wheel
                w.writerow([0,ms/1000,*row])
        self.status=dict(csv_file=str(self.csv),running=True,connected=True,updated_at=dt.datetime.now(dt.timezone.utc).isoformat(),last_sample_at=dt.datetime.now(dt.timezone.utc).isoformat())
        (self.path/'status.json').write_text(json.dumps(self.status))
        self.app=Dashboard(self.path)
    def tearDown(self):
        self.app.reset();self.temp.cleanup()
    def test_incremental_live_export_and_serial_free_phase_mailbox(self):
        state=self.app.state();self.assertEqual(len(state['shots']),1)
        self.assertTrue(state['shots'][0]['valid'])
        self.assertEqual(state['live'][-1][5:],[100,101,102,200,201,202])
        self.assertEqual(len(self.app.state()['shots']),1)
        shot=self.app.detail(state['shots'][0]['id'])
        self.assertEqual(shot['seg']['current_raw'][0],[200,201,202])
        result=self.app.phase(2)
        self.assertEqual(result['phase_centideg'],-200)
        self.assertTrue((self.path/'dial_phase_command.json').exists())
        self.app.export(self.path/'out.csv')
        with (self.path/'out.csv').open() as stream:rows=list(csv.DictReader(stream))
        self.assertEqual(len(rows),3)
        self.assertEqual(rows[0]['before_rpm'],'5750.0')
        with self.assertRaises(ValueError):self.app.phase(20)
        # An unfinished CSV row must wait, not be consumed or duplicated.
        with self.csv.open('a') as stream:stream.write('incomplete')
        self.app.state();offset=self.app.fp.tell();self.app.state()
        self.assertEqual(self.app.fp.tell(),offset)
    def test_pitch_limit_mailbox_and_extension_route(self):
        import re
        extension=(Path(__file__).resolve().parents[1]/'tools/vscode-shot-scope/extension.js').read_text()
        allowed=re.search(r'const allowed = (\[.*?\]);',extension).group(1)
        self.assertIn("'pitch_limit'",allowed)
        for endpoint in (1,2):
            result=self.app.dispatch('pitch_limit',{'endpoint':endpoint})
            self.assertEqual(result['pitch_endpoint'],endpoint)
            command=json.loads((self.path/'dial_phase_command.json').read_text())
            self.assertEqual(command['pitch_endpoint'],endpoint)
        with self.assertRaises(ValueError):self.app.pitch_limit(3)
        self.app.mode='replay'
        with self.assertRaises(ValueError):self.app.pitch_limit(1)

    def test_replay_is_read_only_and_returns_same_metrics(self):
        live=self.app.state();replay=self.app.replay(self.csv)
        self.assertEqual(replay['mode'],'replay')
        self.assertEqual(live['shots'][0]['drop_rpm'],replay['shots'][0]['drop_rpm'])
        with self.assertRaises(ValueError):self.app.phase(2)
        self.app.dispatch('live',{});self.assertEqual(self.app.mode,'live')

if __name__=='__main__':unittest.main()
