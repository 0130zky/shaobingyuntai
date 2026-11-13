#!/usr/bin/env python3
"""VS Code dashboard backend. Local stdin/stdout JSON RPC; no network or serial ownership."""
import argparse
from collections import deque
import csv
import datetime as dt
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
from telemetry_protocol import FIELDS, DIAG_FIELDS, RECORD_FIELDS
from analysis_core import measure_shot, mark_duplicates
from board_clock import BoardClock


def clean(value):
    if isinstance(value,np.ndarray):return clean(value.tolist())
    if isinstance(value,np.generic):return clean(value.item())
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    return value


class Dashboard:
    def __init__(self,directory):
        self.directory=Path(directory)
        self.samples=deque(maxlen=60000)
        self.shots=deque(maxlen=200)
        self.pending=deque()
        self.fp=None;self.path=None;self.header=None
        self.clock=0.;self.previous_ms=None;self.previous_count=None;self.board_clock=BoardClock()
        self.sequence=0;self.mode='live';self.status={};self.loading=False

    def reset(self):
        if self.fp:self.fp.close()
        self.samples.clear();self.shots.clear();self.pending.clear()
        self.fp=None;self.path=None;self.previous_ms=None;self.previous_count=None
        self.clock=0.;self.board_clock=BoardClock()

    def append(self,values):
        self.clock=self.board_clock.advance(values[7])
        if not self.board_clock.changed:return
        if self.board_clock.reset:
            self.pending.clear();self.previous_count=None;self.samples.clear()
        self.samples.append((self.clock,values))
        count=int(values[5])
        if self.previous_count is not None and 0<(count-self.previous_count)%65536<32768:
            self.pending.append((self.clock,count))
        self.previous_count=count

    def finish(self):
        while self.pending and self.clock>=self.pending[0][0]+.85:
            stamp,count=self.pending.popleft()
            window=[row for row in self.samples if stamp-.801<=row[0]<=stamp+.801]
            if not window:continue
            t=np.array([row[0] for row in window]);d=np.array([row[1] for row in window])
            index=int(np.argmin(abs(t-stamp)))
            shot=measure_shot(t,d,index,float(d[index,3]))
            self.sequence+=1
            shot.update(id=self.sequence,count=count)
            if d.shape[1]>25 and shot.get('onset_t') is not None:
                shot['diagnostic_seg']=dict(t=(t-shot['onset_t']).tolist(),fields=DIAG_FIELDS+['diag_age_ms'],values=d[:,25:].tolist())
            self.shots.append(clean(shot))
            mark_duplicates(list(self.shots))

    def read_live(self):
        if self.mode!='live':return
        try:self.status=json.loads((self.directory/'status.json').read_text(encoding='utf-8'))
        except (OSError,ValueError):self.status={}
        path=self.status.get('csv_file')
        if not path:return
        if path!=self.path:
            if self.fp:self.fp.close()
            self.fp=open(path,newline='')
            self.header=next(csv.reader([self.fp.readline()]))
            # Restore a recent window quickly; full files remain available via replay.
            if Path(path).stat().st_size>8_000_000:
                self.fp.seek(Path(path).stat().st_size-8_000_000)
                self.fp.readline()
            self.path=path
        self.loading=True
        for _ in range(8192):
            offset=self.fp.tell();line=self.fp.readline()
            if not line or not line.endswith('\n'):
                self.fp.seek(offset);self.loading=False;break
            row=next(csv.reader([line]))
            if len(row)!=len(self.header):continue
            try:
                mapping=dict(zip(self.header,row))
                values=[float(mapping.get(name) or ('nan' if name not in FIELDS else 0)) for name in RECORD_FIELDS]
                if all(math.isfinite(v) for v in values[:25]):self.append(values)
            except (ValueError,KeyError):continue
        self.finish()

    def state(self):
        self.read_live()
        latest=self.samples[-1][1] if self.samples else None
        recent=[row for row in self.samples if row[0]>=self.clock-8]
        step=max(1,(len(recent)+799)//800)
        live=[[stamp,*values[:4],*values[10:16]] for stamp,values in recent[::step]]
        age=None
        if self.status.get('last_sample_at'):
            age=max(0,time.time()-dt.datetime.fromisoformat(self.status['last_sample_at']).timestamp())
        return clean(dict(mode=self.mode,loading=self.loading,status=self.status,
                          data_age_s=age,latest=dict(zip(RECORD_FIELDS,latest)) if latest else None,
                          shots=[{k:v for k,v in shot.items() if k not in ('seg','diagnostic_seg')} for shot in self.shots],
                          live=live,source=self.path,diagnostic_live=[[stamp,*values[25:]] for stamp,values in recent[::step]]))

    def replay(self,path):
        path=Path(path)
        if path.suffix.lower()!='.csv':raise ValueError('请选择CSV记录')
        self.reset();self.mode='replay';self.path=str(path)
        with path.open(newline='') as stream:
            for row in csv.DictReader(stream):
                try:values=[float(row.get(name) or ('nan' if name not in FIELDS else 0)) for name in RECORD_FIELDS]
                except (ValueError,TypeError):continue
                if all(math.isfinite(v) for v in values[:25]):
                    self.append(values)
                    if self.pending and self.clock>=self.pending[0][0]+.85:self.finish()
        return self.state()

    def detail(self,identity):
        for shot in self.shots:
            if shot['id']==identity:return shot
        raise ValueError('该发记录不存在，请重新选择')

    def pitch_limit(self,endpoint):
        if endpoint not in (1,2):raise ValueError('无效限位')
        if self.mode!='live':raise ValueError('回放模式不能标定')
        self.read_live()
        stamp=dt.datetime.fromisoformat(self.status.get('updated_at','1970-01-01T00:00:00+00:00')).timestamp()
        if not self.status.get('connected') or time.time()-stamp>3:raise ValueError('后台采集未连接')
        command=dict(pitch_endpoint=endpoint,created_unix_s=time.time())
        temp=self.directory/'dial_phase_command.tmp'
        temp.write_text(json.dumps(command));temp.replace(self.directory/'dial_phase_command.json')
        return command

    def phase(self,clockwise):
        clockwise=float(clockwise)
        if self.mode!='live':raise ValueError('回放模式不能下发调节')
        self.read_live()
        stamp=dt.datetime.fromisoformat(self.status.get('updated_at','1970-01-01T00:00:00+00:00')).timestamp()
        if not self.status.get('running') or not self.status.get('connected') or time.time()-stamp>3:
            raise ValueError('后台采集未连接')
        if not math.isfinite(clockwise) or abs(clockwise)>10:raise ValueError('偏移范围为±10°')
        command=dict(phase_centideg=round(-clockwise*100),created_unix_s=time.time())
        temp=self.directory/'dial_phase_command.tmp'
        temp.write_text(json.dumps(command));temp.replace(self.directory/'dial_phase_command.json')
        return command

    def export(self,path):
        metrics=('bases','mins','drop_rpm','drop','after_rpm','after_delta','recovery_ms','overshoot_rpm','onset_ms','min_ms')
        with Path(path).open('w',newline='') as stream:
            writer=csv.writer(stream)
            writer.writerow(['count','speed_mps','wheel','before_rpm','min_rpm','drop_rpm','drop_pct','after_rpm','after_delta_rpm','recovery_ms','overshoot_rpm','onset_ms','minimum_ms','valid','note'])
            for shot in self.shots:
                for w,name in enumerate(('12点','4点','8点')):
                    values=[shot[key][w] for key in metrics]
                    if values[3] is not None:values[3]*=100
                    writer.writerow([shot['count'],shot['speed'],name,*values,shot['valid'],shot['note']])
        return dict(path=str(path),shots=len(self.shots))

    def dispatch(self,method,params):
        if method=='state':return self.state()
        if method=='shot':return self.detail(int(params['id']))
        if method=='replay':return self.replay(params['path'])
        if method=='live':self.reset();self.mode='live';return self.state()
        if method=='pitch_limit':return self.pitch_limit(params['endpoint'])
        if method=='phase':return self.phase(params['clockwise'])
        if method=='export':return self.export(params['path'])
        raise ValueError('Unknown method')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,default=Path(__file__).parent/'records/monitor')
    args=parser.parse_args();dashboard=Dashboard(args.directory)
    for line in sys.stdin:
        request={}
        try:
            request=json.loads(line)
            result=dashboard.dispatch(request['method'],request.get('params',{}))
            response=dict(id=request.get('id'),result=clean(result))
        except Exception as exc:
            response=dict(id=request.get('id'),error=str(exc))
        print(json.dumps(response,ensure_ascii=False,allow_nan=False),flush=True)
