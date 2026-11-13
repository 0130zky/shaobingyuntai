#!/usr/bin/env python3
"""Offline evaluation of wheel RPM tracking, loaded synchronization and recovery.
Reads existing records only. Current is reported as raw counts, not assumed amperes.
"""
import argparse
from bisect import bisect_right
from collections import Counter,defaultdict
import csv
import datetime as dt
import json
from pathlib import Path
import statistics
import numpy as np
from telemetry_protocol import FIELDS
from board_clock import collapse_times


def finite(value):
    if isinstance(value,np.ndarray):return finite(value.tolist())
    if isinstance(value,np.generic):return finite(value.item())
    if isinstance(value,float) and not np.isfinite(value):return None
    if isinstance(value,dict):return {key:finite(v) for key,v in value.items()}
    if isinstance(value,(tuple,list)):return [finite(v) for v in value]
    return value


def recovery_time(t,rpm,minimum,target,hold=.05,tolerance=100):
    """First in-band interval with a complete hold; truncated tails never pass."""
    for start in range(minimum,len(t)):
        end=int(np.searchsorted(t,t[start]+hold))
        if end>=len(t):break
        if np.all(np.abs(rpm[start:end+1]-target)<=tolerance):
            return float((t[start]-t[minimum])*1000)
    return float('nan')


def evaluate_window(t,data,event_index):
    """Assess one isolated load; selection does not reject mere RPM ripple."""
    t=np.asarray(t);data=np.asarray(data);target=float(data[event_index,3]);event_time=t[event_index]
    if len(t)<40 or np.max(np.diff(t))>.020:return dict(usable=False,reason='采样不足或存在超过20ms断流')
    if target<1000:return dict(usable=False,reason='目标转速过低')
    rpm=data[:,:3];mean=rpm.mean(axis=1)
    initial=(t>=event_time-.65)&(t<=event_time-.40)
    if initial.sum()<10:return dict(usable=False,reason='发前窗口不足')
    rough=np.median(rpm[initial],axis=0)
    search=np.flatnonzero((t>=event_time-.35)&(t<=event_time+.08))
    if not len(search):return dict(usable=False,reason='过弹窗口不足')
    minimum=int(search[np.argmin(mean[search])]);onset=minimum
    while onset>0 and mean[onset-1]<rough.mean()-max(100,.02*rough.mean()):onset-=1
    origin=float(t[onset]);pre=(t>=origin-.25)&(t<=origin-.05)
    if pre.sum()<10:return dict(usable=False,reason='掉速前基线不足')
    bases=np.median(rpm[pre],axis=0)
    if (bases.mean()-mean[minimum])/target<.03:return dict(usable=False,reason='未检测到独立明显掉速')
    fixed=(t>=origin-.25)&(t<=origin+.12)
    if np.ptp(data[fixed,3])>1:return dict(usable=False,reason='过弹前后正在调目标转速')
    if np.max(abs(bases-target))>75:return dict(usable=False,reason='过弹前基线尚未到速')
    near=np.flatnonzero((t>=origin)&(t<=origin+.10))
    minima=[int(near[np.argmin(rpm[near,w])]) for w in range(3)]
    loaded=(t>=origin)&(t<=origin+.12)
    indices=np.flatnonzero(loaded);common_min=int(indices[np.argmin(mean[indices])])
    # Stop recovery attribution at a target change. Do not credit commanded deceleration.
    end=int(np.searchsorted(t,origin+.70,side='right'))
    changes=np.flatnonzero((t>=origin)&(abs(data[:,3]-target)>1))
    if len(changes):end=min(end,int(changes[0]))
    rec=[recovery_time(t[:end],rpm[:end,w],minima[w],target) if minima[w]<end else float('nan') for w in range(3)]
    drop=[float(bases[w]-rpm[minima[w],w]) for w in range(3)]
    peak_cmd=np.max(abs(data[loaded,10:13]),axis=0)
    peak_current=np.max(abs(data[loaded,13:16]),axis=0)
    peaks=[int(indices[np.argmax(abs(data[loaded,10+w]))]) for w in range(3)]
    result=dict(usable=True,target_rpm=target,bases=bases.tolist(),minimum_rpm=[float(rpm[minima[w],w]) for w in range(3)],
                drop_rpm=drop,drop_pct=[100*drop[w]/bases[w] for w in range(3)],
                common_min_spread_rpm=float(np.ptp(rpm[common_min])),
                peak_loaded_spread_rpm=float(np.ptp(rpm[loaded],axis=1).max()),
                recovery_ms=rec,recovery_spread_ms=float(max(rec)-min(rec)) if np.all(np.isfinite(rec)) else float('nan'),
                peak_command_raw=peak_cmd.tolist(),peak_command_pct=(100*peak_cmd/16384).tolist(),
                peak_feedback_current_raw=peak_current.tolist(),
                saturated_fraction=np.mean(abs(data[loaded,10:13])>=.98*16384,axis=0).tolist(),
                command_peak_delay_ms=[float((t[j]-origin)*1000) for j in peaks],
                falling_duration_ms=float((t[common_min]-origin)*1000),
                overshoot_rpm=[max(0,float(rpm[minima[w]:end,w].max()-target)) if minima[w]<end else float('nan') for w in range(3)],
                baseline_ripple_rpm=np.ptp(rpm[pre],axis=0).tolist(),
                baseline_error_mean=np.mean(rpm[pre]-target,axis=0).tolist(),
                sample_interval_ms=np.diff(t)*1000)
    result['_steady_error']=rpm[pre]-target
    result['_steady_spread']=np.ptp(rpm[pre],axis=1)
    clip=(t>=origin-.02)&(t<=origin+.15)
    result['_trace']=dict(ms=(t[clip]-origin)*1000,rpm=rpm[clip],command=data[clip,10:13],current=data[clip,13:16])
    return result


def distribution(values):
    a=np.asarray(values,dtype=float);a=a[np.isfinite(a)]
    if not len(a):return dict(n=0,median=None,p95=None,max=None)
    return dict(n=len(a),median=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(a.max()))


def aggregate(shots):
    errors=np.concatenate([s['_steady_error'] for s in shots]);spread=np.concatenate([s['_steady_spread'] for s in shots])
    wheels=[]
    for w,name in enumerate(('12点轮','4点轮','8点轮')):
        err=errors[:,w]
        wheels.append(dict(wheel=name,mean_error_rpm=float(err.mean()),rmse_rpm=float(np.sqrt(np.mean(err**2))),
                           abs_error_p95_rpm=float(np.percentile(abs(err),95)),max_abs_error_rpm=float(abs(err).max()),
                           within20_pct=float(np.mean(abs(err)<=20)*100),within50_pct=float(np.mean(abs(err)<=50)*100),
                           ripple=distribution([s['baseline_ripple_rpm'][w] for s in shots]),
                           drop_rpm=distribution([s['drop_rpm'][w] for s in shots]),drop_pct=distribution([s['drop_pct'][w] for s in shots]),
                           recovery_ms=distribution([s['recovery_ms'][w] for s in shots]),
                           overshoot_rpm=distribution([s['overshoot_rpm'][w] for s in shots]),
                           command_peak_pct=distribution([s['peak_command_pct'][w] for s in shots]),
                           saturated_shots=sum(s['saturated_fraction'][w]>0 for s in shots),
                           peak_delay_ms=distribution([s['command_peak_delay_ms'][w] for s in shots])))
    return dict(n=len(shots),wheels=wheels,steady_spread=distribution(spread),
                all_wheels_within50_pct=float(np.mean(np.all(abs(errors)<=50,axis=1))*100),
                all_wheels_within20_pct=float(np.mean(np.all(abs(errors)<=20,axis=1))*100),
                steady_spread_within30_pct=float(np.mean(spread<=30)*100),
                common_min_spread=distribution([s['common_min_spread_rpm'] for s in shots]),
                peak_loaded_spread=distribution([s['peak_loaded_spread_rpm'] for s in shots]),
                recovery_spread_ms=distribution([s['recovery_spread_ms'] for s in shots]),
                falling_duration_ms=distribution([s['falling_duration_ms'] for s in shots]),
                sample_interval_ms=distribution(np.concatenate([s['sample_interval_ms'] for s in shots])))


def read_windows(session,events):
    windows=[(dt.datetime.fromisoformat(e['time']).timestamp()-.85,dt.datetime.fromisoformat(e['time']).timestamp()+.90) for e in events]
    starts=[a for a,b in windows];buffers=[[] for _ in events]
    for path in sorted(session.glob('telemetry_*.csv')):
        with path.open() as stream:
            for row in csv.DictReader(stream):
                try:stamp=float(row['host_unix_s'])
                except (ValueError,TypeError):continue
                j=bisect_right(starts,stamp)-1
                while j>=0 and windows[j][1]>=stamp:
                    try:values=[float(row.get(field,0)) for field in FIELDS]
                    except (ValueError,TypeError):break
                    buffers[j].append((stamp,values));j-=1
    return buffers


def write_report(report,out):
    p=report['primary'];f=lambda x:'—' if x is None else f'{x:.1f}'
    lines=['# 三摩擦轮转速稳定性评估','',f"数据：{report['session']}",f"评估起点：{report['since']}",
           f"裁判消息{report['input_records']}条；相邻成对消息{report['paired_records']}条单列；有效独立加载样本{report['usable_records']}条。",
           f"主比较组：共同目标{report['primary_target_rpm']:.0f}rpm，{p['n']}个样本。不同上电段仍分别保留，合并值不用于宣称某次调参有因果改善。",'',
           '## 1. 发弹前稳态','',
           '|轮位|平均误差 rpm|RMSE rpm|绝对误差P95 rpm|每发峰峰值中位数 rpm|±20rpm样本占比|±50rpm样本占比|',
           '|---|---:|---:|---:|---:|---:|---:|']
    for w in p['wheels']:lines.append(f"|{w['wheel']}|{f(w['mean_error_rpm'])}|{f(w['rmse_rpm'])}|{f(w['abs_error_p95_rpm'])}|{f(w['ripple']['median'])}|{f(w['within20_pct'])}%|{f(w['within50_pct'])}%|")
    lines += ['',f"三轮同时处于目标±50rpm：{f(p['all_wheels_within50_pct'])}%；同时±20rpm：{f(p['all_wheels_within20_pct'])}%。",
              f"同刻轮间最大差：中位{f(p['steady_spread']['median'])}rpm，P95 {f(p['steady_spread']['p95'])}rpm；轮间差≤30rpm占比{f(p['steady_spread_within30_pct'])}%。",'',
              '## 2. 过弹与恢复','',
              '|轮位|掉速中位数 rpm|掉速中位数 %|恢复中位数 ms|恢复P95 ms|最慢恢复 ms|电流指令峰值中位数 %满量程|触及98%上限样本数|',
              '|---|---:|---:|---:|---:|---:|---:|---:|']
    for w in p['wheels']:lines.append(f"|{w['wheel']}|{f(w['drop_rpm']['median'])}|{f(w['drop_pct']['median'])}|{f(w['recovery_ms']['median'])}|{f(w['recovery_ms']['p95'])}|{f(w['recovery_ms']['max'])}|{f(w['command_peak_pct']['median'])}|{w['saturated_shots']}|")
    lines += ['',f"平均轮速最低点的同刻轮间差：中位{f(p['common_min_spread']['median'])}rpm，P95 {f(p['common_min_spread']['p95'])}rpm。",
              f"每发0～120ms窗口内最大轮间差：中位{f(p['peak_loaded_spread']['median'])}rpm，P95 {f(p['peak_loaded_spread']['p95'])}rpm，最大{f(p['peak_loaded_spread']['max'])}rpm。",
              f"三轮恢复时间差：中位{f(p['recovery_spread_ms']['median'])}ms，P95 {f(p['recovery_spread_ms']['p95'])}ms，最大{f(p['recovery_spread_ms']['max'])}ms。",
              f"遥测掉速起点至最低点：中位{f(p['falling_duration_ms']['median'])}ms。恢复从各轮独立最低点计时，回到共同目标±100rpm且保持50ms；尾部不足不计成功。",'',
              '## 3. 控制实现与测量边界','',
              '以下控制参数为2026-10-28源码核查快照，不从日志推断；后续改固件需重新核查。','',
              '- 当前实现：三路独立PI（Kp=10、Ki=20），1ms输出低通；轮间差速参考补偿0.30倍、±100rpm封顶，三轮共同放缩保持补偿和为零。',
              '- 同步补偿要求三路反馈年龄≤5ms、时间戳差≤1ms；现有25通道没有rx_tick、实际PID参考、补偿量或PID周期，无法统计补偿真正生效的比例。',
              '- PID任务使用相对osDelay(3)，周期还包含运算/调度耗时；本报告的采样间隔不是实际PID执行周期。',
              f"- 有效评估窗口遥测间隔中位{f(p['sample_interval_ms']['median'])}ms、P95 {f(p['sample_interval_ms']['p95'])}ms。几毫秒掉速只有少数采样点；CAN反馈各轮也不保证同一物理时刻，极值和起点差有采样误差。",
              '- 电流仅按原始指令满量程16384计百分比；不假设反馈原始值与实际安培的换算。',
              '- 轮速基线误差、瞬时波动、负载掉速和恢复分开评价；掉速本身是能量转移的表现，不以“完全不掉速”为验收条件。',
              '- ±50rpm是当前代码的就绪容差，不是16.45±0.05m/s的保证。若仅做近似比例估算，0.05/16.45约0.304%，在5782rpm约为18rpm，可作为更紧稳态目标的量级参考。',
              '- 本次只读取历史记录并生成评估，不修改或烧录电机参数。','',
              '## 4. 排除与分组','',json.dumps(report['excluded_reasons'],ensure_ascii=False),
              '', '|目标rpm|有效加载样本|弹速均值m/s|最低～最高m/s|', '|---:|---:|---:|---:|']
    for rpm,g in report['by_target'].items():lines.append(f"|{rpm}|{g['n']}|{g['bullet_mean']:.3f}|{g['bullet_min']:.3f}～{g['bullet_max']:.3f}|")
    lines += ['', '弹速列仅用于关联，不将不同速度、相位和上电状态的记录拟合成单一转速标定曲线。', '',
              '## 5. 评估结论与下一步', '',
              '- 平均目标跟踪误差小不等于瞬时稳定。先按稳态误差P95、峰峰值和三轮同时到速率定位最弱轮，避免仅比较每发平均转速。',
              '- 受载后应比较同一采样时刻三轮差速；三轮各自最低转速相近，不代表过弹与恢复过程同步。120ms窗口包含弹丸离开后的恢复段，不能把其中最大差直接当成弹丸接触期间的差速。',
              '- 先补齐实际控制周期、三轮反馈年龄和同步补偿量，确认时间一致性及同步门槛生效情况；再针对稳态波动最大的轮做小步PI对照。指令已接近满量程时，不直接整体增大Kp或叠加大幅前馈。',
              '- 后续对照固定一个共同目标转速与上电状态，每轮同时比较发前波动、受载差速、恢复P95和接近限幅次数；不要只用平均弹速决定优化是否有效。',
              '- 恢复时间统计仅使用观察窗内完整满足保持条件的样本；各轮有效样本数见evaluation.json中的recovery_ms.n，未完成恢复者不按0ms计算。', '',
              '详细数据：evaluation.json、shots.csv；图：wheel_evaluation.png。']
    (out/'report.md').write_text('\n'.join(lines)+'\n')


def plot(report,shots,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors=['#e87555','#21a183','#527dde'];fig,axes=plt.subplots(2,2,figsize=(12,8))
    errors=np.concatenate([s['_steady_error'] for s in shots])
    axes[0,0].boxplot([errors[:,i] for i in range(3)],tick_labels=['12h','4h','8h'],showfliers=False)
    axes[0,0].axhline(20,ls='--',color='gray',lw=.8);axes[0,0].axhline(-20,ls='--',color='gray',lw=.8)
    axes[0,0].set_title('Pre-shot RPM error (instantaneous samples)');axes[0,0].set_ylabel('RPM - target')
    for w in range(3):
        axes[0,1].plot([s['drop_rpm'][w] for s in shots],'.',color=colors[w],label=['12h','4h','8h'][w])
        axes[1,0].plot([s['recovery_ms'][w] for s in shots],'.',color=colors[w],label=['12h','4h','8h'][w])
    axes[0,1].set_title('Per-shot speed drop');axes[0,1].set_ylabel('RPM drop')
    axes[1,0].set_title('Recovery to target +/-100 RPM, held 50 ms');axes[1,0].set_ylabel('ms from each wheel minimum')
    axes[1,1].plot([s['common_min_spread_rpm'] for s in shots],label='At mean-speed nadir')
    axes[1,1].plot([s['peak_loaded_spread_rpm'] for s in shots],label='Maximum within 120 ms',alpha=.8)
    axes[1,1].set_title('Same-sample wheel-to-wheel spread');axes[1,1].set_ylabel('RPM max - min')
    for ax in axes.ravel():ax.grid(alpha=.25)
    for ax in (axes[0,1],axes[1,0],axes[1,1]):ax.legend(fontsize=8);ax.set_xlabel('Selected isolated-load index')
    fig.suptitle(f"Three-wheel evaluation | target {report['primary_target_rpm']:.0f} RPM | {len(shots)} loads")
    fig.tight_layout(rect=(0,0,1,.95));fig.savefig(out/'wheel_evaluation.png',dpi=160);plt.close(fig)


def run(session,since,target,out):
    events=[json.loads(line) for line in (session/'events.jsonl').read_text().splitlines()]
    messages=[e for e in events if e.get('event')=='shot_message' and dt.datetime.fromisoformat(e['time'])>dt.datetime.fromisoformat(since)]
    paired=set()
    for i,(a,b) in enumerate(zip(messages,messages[1:])):
        gap=(dt.datetime.fromisoformat(b['time'])-dt.datetime.fromisoformat(a['time'])).total_seconds()
        if 0<gap<.35 and a['frame']['dial_target_deg']==b['frame']['dial_target_deg']:paired.update((i,i+1))
    selected=[e for i,e in enumerate(messages) if i not in paired];buffers=read_windows(session,selected)
    results=[];reasons=Counter();block=0;last_counter=None
    for event,rows in zip(selected,buffers):
        count=int(event['frame']['shot_count_u16'])
        if last_counter is not None and count<last_counter:block+=1
        last_counter=count
        if not rows:reasons['无采样窗口']+=1;continue
        data=np.array([values for host,values in rows]);host=np.array([host for host,values in rows]);ms=data[:,7].astype(np.int64)
        times,keep=collapse_times(ms);t=np.asarray(times);data=data[keep];host=host[keep];ms=ms[keep]
        stamp=dt.datetime.fromisoformat(event['time']).timestamp()
        matches=np.flatnonzero((ms==int(event['frame']['sample_ms_u16']))&(data[:,5]==count))
        index=int(matches[np.argmin(abs(host[matches]-stamp))]) if len(matches) else int(np.argmin(abs(host-stamp)))
        result=evaluate_window(t,data,index)
        if not result['usable']:reasons[result['reason']]+=1;continue
        result.update(time=event['time'],counter=count,block=block,bullet_mps=event['frame']['bullet_speed_mps'])
        results.append(result)
    primary=[s for s in results if s['target_rpm']==target]
    if not primary:raise ValueError('指定目标转速无有效样本')
    by_target={};by_block={}
    for key,g in ((rpm,[s for s in results if s['target_rpm']==rpm]) for rpm in sorted(set(s['target_rpm'] for s in results))):
        values=[s['bullet_mps'] for s in g];by_target[str(int(key))]=dict(n=len(g),bullet_mean=statistics.mean(values),bullet_min=min(values),bullet_max=max(values))
    for key in sorted(set(s['block'] for s in primary)):
        by_block[str(key)]=aggregate([s for s in primary if s['block']==key])
    report=dict(session=str(session.resolve()),since=since,input_records=len(messages),paired_records=len(paired),
                usable_records=len(results),excluded_reasons=dict(reasons),primary_target_rpm=target,
                primary=aggregate(primary),by_block=by_block,by_target=by_target,
                shots=[{k:v for k,v in s.items() if not k.startswith('_')} for s in results])
    out.mkdir(parents=True,exist_ok=True)
    (out/'evaluation.json').write_text(json.dumps(finite(report),ensure_ascii=False,indent=2)+'\n')
    with (out/'shots.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['time','counter','target_rpm','wheel','base_rpm','minimum_rpm','drop_rpm','recovery_ms','peak_command_pct','peak_loaded_spread_rpm'])
        for s in results:
            for w,name in enumerate(('12h','4h','8h')):writer.writerow([s['time'],s['counter'],s['target_rpm'],name,s['bases'][w],s['minimum_rpm'][w],s['drop_rpm'][w],s['recovery_ms'][w],s['peak_command_pct'][w],s['peak_loaded_spread_rpm']])
    write_report(finite(report),out);plot(report,primary,out)
    print(json.dumps(finite({k:report[k] for k in ('input_records','paired_records','usable_records','excluded_reasons','primary_target_rpm','primary')}),ensure_ascii=False,indent=2))
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('session',type=Path)
    parser.add_argument('--since',required=True);parser.add_argument('--target',type=float,default=5782)
    parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    run(args.session,args.since,args.target,args.out)
