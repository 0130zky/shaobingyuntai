"""Per-wheel shot measurements in RPM, aligned by a detected speed drop.
The origin is a telemetry threshold, not a physical projectile-contact sensor.
"""
import numpy as np


def measure_shot(t, d, i, target_rpm):
    te=float(t[i]);nan=float('nan')
    out=dict(i=i,t=te,speed=float(d[i,4]),valid=False,dup=False,note='',
             base=nan,bases=[nan]*3,mins=[nan]*3,drop=[nan]*3,drop_rpm=[nan]*3,
             after_rpm=[nan]*3,after_delta=[nan]*3,overshoot_rpm=[nan]*3,
             recovery_ms=[nan]*3,onset_ms=[nan]*3,min_ms=[nan]*3,
             contact_ms=nan,contact_wheel=-1,
             drop_mean=nan,recov_ms=nan,peak_amp=[nan]*3,dial_dps=nan,
             ready=int(d[i,24]),latency_ms=nan,seg=None)
    lo=np.searchsorted(t,te-.8);hi=np.searchsorted(t,te+.8,side='right')
    tt=np.asarray(t[lo:hi]);dd=np.asarray(d[lo:hi]);rpm=dd[:,:3].astype(float)
    if len(tt)<20 or np.any(np.diff(tt)<=0) or np.max(np.diff(tt))>.020:
        out['note']='数据不足或采样断流';return out
    baseline=(tt>=te-.8)&(tt<=te-.5)
    if baseline.sum()<10:
        out['note']='发弹前基线不足';return out
    bases=np.median(rpm[baseline],axis=0)
    if np.min(bases)<500:
        out['note']='轮子未起转';return out
    search=np.flatnonzero((tt>=te-.4)&(tt<=te+.15))
    if not len(search):
        out['note']='过弹窗口不足';return out
    mean=rpm.mean(axis=1);k=int(search[np.argmin(mean[search])])
    s=k;floor=bases.mean()-max(100.,.02*bases.mean())
    while s>0 and mean[s-1]<floor:s-=1
    # Re-estimate each baseline immediately before the drop, excluding its last 50 ms.
    pre=(tt>=tt[s]-.30)&(tt<=tt[s]-.05)
    if pre.sum()<10:
        out['note']='掉速前基线不足';return out
    bases=np.median(rpm[pre],axis=0)
    base=float(bases.mean())
    if np.min(bases)<500 or 1.-mean[k]/base<.03:
        out['note']='未检测到掉速';return out
    if np.any(np.ptp(rpm[pre],axis=0)>100) or np.ptp(dd[pre,3])>1:
        out['note']='发弹前轮速未稳定';return out
    # A setpoint change makes recovery/drop attribution ambiguous.
    if np.ptp(dd[s:,3])>1:
        out['note']='测量窗口目标转速变化';return out
    near=np.flatnonzero((tt>=tt[k]-.035)&(tt<=tt[k]+.035))
    minima=[int(near[np.argmin(rpm[near,w])]) for w in range(3)]
    out.update(base=base,bases=bases.tolist(),onset_t=float(tt[s]),
               mins=[float(rpm[minima[w],w]) for w in range(3)],
               latency_ms=float((te-tt[s])*1000),valid=True)
    for w,kw in enumerate(minima):
        out['drop_rpm'][w]=float(bases[w]-rpm[kw,w])
        out['drop'][w]=out['drop_rpm'][w]/bases[w]
        sw=kw
        while sw>0 and rpm[sw-1,w]<bases[w]-max(100.,.02*bases[w]):sw-=1
        out['onset_ms'][w]=float((tt[sw]-tt[s])*1000)
        out['min_ms'][w]=float((tt[kw]-tt[s])*1000)
        # Require a full 50 ms in a two-sided band; do not accept a truncated tail.
        for j in range(kw,len(tt)):
            end=int(np.searchsorted(tt,tt[j]+.05))
            if end>=len(tt):break
            if np.all(np.abs(rpm[j:end+1,w]-bases[w])<=100):
                out['recovery_ms'][w]=float((tt[j]-tt[kw])*1000);break
        post=(tt>=tt[s]+.4)&(tt<=tt[s]+.5)
        if tt[-1]>=tt[s]+.5 and post.sum()>=10:
            out['after_rpm'][w]=float(np.mean(rpm[post,w]))
            out['after_delta'][w]=out['after_rpm'][w]-bases[w]
        tail=(tt>=tt[kw])&(tt<=tt[s]+.5)
        out['overshoot_rpm'][w]=max(0.,float(np.max(rpm[tail,w])-bases[w]))
        end=int(np.searchsorted(tt,tt[kw]+.3))
        out['peak_amp'][w]=float(np.max(np.abs(dd[s:end,13+w])))*20./16384.
    # 最早接触：三轮中最早开始掉速的那一轮，时间相对本段零点（三轮平均掉速起点）。
    # 单轮掉速起点可能早于均值起点，因此该值通常 ≤0。
    if len(out['onset_ms'])==3 and np.all(np.isfinite(out['onset_ms'])):
        cw=int(np.argmin(out['onset_ms']))
        out['contact_wheel']=cw
        out['contact_ms']=float(out['onset_ms'][cw])
    out['drop_mean']=float(np.mean(out['drop']))
    out['recov_ms']=max(out['recovery_ms']) if np.all(np.isfinite(out['recovery_ms'])) else nan
    if not np.all(np.isfinite(out['recovery_ms'])):out['note']='窗口内未全部恢复'
    out['dial_dps']=float(np.max(np.abs(dd[s:min(len(tt),k+100),18])))
    out['ready']=int(dd[max(0,np.searchsorted(tt,tt[s]-.1)-1),24])
    seg=(tt>=tt[s]-.3)&(tt<=tt[s]+.6)
    out['seg']=dict(t=tt[seg]-tt[s],rpm=rpm[seg].copy(),
                    amp=dd[seg,13:16].astype(float)*20./16384.,
                    current_raw=dd[seg,13:16].astype(float).copy(),
                    command_raw=dd[seg,10:13].astype(float).copy(),
                    base=base,bases=bases.copy())
    return out

DUP_GAP = .70

def analyze_all(t, d, target_rpm):
    """按射击计数变化切出每一发，返回 [shot dict]，时间升序。"""
    if len(t) < 2:
        return []
    cnt = d[:, 5].astype(np.int64)
    delta = np.diff(cnt) % 65536
    idx = np.where((delta > 0) & (delta < 32768))[0] + 1
    shots = []
    for i in idx:
        shots.append(measure_shot(t, d, int(i), target_rpm))
    mark_duplicates(shots)
    return shots



def mark_duplicates(shots):
    """同一掉速事件被相同弹速短间隔重复上报时标记；独立掉速仍保留。"""
    prev = None
    for s in shots:
        if prev is not None and (s['t'] - prev['t']) < DUP_GAP and \
                abs(s.get('onset_t', s['t']) - prev.get('onset_t', prev['t'])) < .03 and \
                abs(s['speed'] - prev['speed']) < 2e-3:
            s['dup'] = True
            s['note'] = '疑似重复上报'
        prev = s if not s['dup'] else prev


def shot_stats(shots, lo, hi):
    vals = [s['speed'] for s in shots if s['valid'] and not s['dup']]
    if not vals:
        return None
    a = np.asarray(vals)
    return {
        'n': len(a),
        'mean': float(a.mean()),
        'std': float(a.std(ddof=1)) if len(a) > 1 else 0.0,
        'min': float(a.min()),
        'max': float(a.max()),
        'hit': int(((a >= lo) & (a <= hi)).sum()),
    }


