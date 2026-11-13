#!/usr/bin/env python3
"""Compare three-wheel telemetry around the detected speed drop (not a contact sensor).
Run with .venv/bin/python; writes JSON metrics and a PNG without touching hardware.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from board_clock import collapse_times, signed_delta_ms
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def analyze(session, first, last, output):
    rows=[]
    for file in sorted(session.glob('telemetry_*.csv')):
        with file.open() as stream:
            rows.extend(csv.DictReader(stream))
    if not rows:
        raise ValueError('No telemetry rows')
    times,keep=collapse_times([float(row['sample_ms_u16']) for row in rows])
    rows=[rows[index] for index in keep]
    def column(name):
        return np.array([float(row[name]) for row in rows])
    raw=column('sample_ms_u16').astype(np.int64)
    delta=np.diff(raw)%65536
    breaks=delta>5000
    segment=np.r_[0,np.cumsum(breaks)]
    times=np.asarray(times)
    rpm=np.array([column('rpm_'+w) for w in ('12h','4h','8h')]).T
    command=np.array([column('cmd_'+w+'_raw') for w in ('12h','4h','8h')]).T
    current=np.array([column('current_'+w+'_raw') for w in ('12h','4h','8h')]).T
    column_target=column('target_rpm')
    counts=column('shot_count_u16').astype(np.int64)
    changes=np.flatnonzero(np.r_[False,np.diff(counts)!=0])
    results=[]
    figures,axes=plt.subplots(3,3,figsize=(13,9),sharex=True)
    for index in changes:
        count=int(counts[index])
        if not first<=count<=last:
            continue
        received=times[index]-signed_delta_ms(raw[index],float(rows[index]['shot_rx_ms_u16']))/1000
        baseline=(times>=received-.4)&(times<=received-.2)&(segment==segment[index])
        target=float(rows[index]['target_rpm'])
        window=(times>=received-.3)&(times<=received+.08)&(segment==segment[index])&(column_target==target)
        ids=np.flatnonzero(window)
        if baseline.sum()<10 or len(ids)<20 or target<=0:
            continue
        base=rpm[baseline].mean(axis=0)
        mean=rpm[ids].mean(axis=1)
        minimum=int(np.argmin(mean))
        onset=minimum
        while onset>0 and mean[onset-1]<base.mean()-100:
            onset-=1
        if onset==0 or base.mean()-mean[minimum]<300:
            continue
        origin=times[ids[onset]]
        onset_each=[]
        minimum_each=[]
        for wheel in range(3):
            k=int(np.argmin(rpm[ids,wheel]))
            j=k
            while j>0 and rpm[ids[j-1],wheel]<base[wheel]-100:
                j-=1
            onset_each.append(round((times[ids[j]]-origin)*1000,2))
            minimum_each.append(round((times[ids[k]]-origin)*1000,2))
        # All wheels compared over the same falling interval; raw current units avoid
        # assuming an unverified amperes-per-count conversion for this ESC firmware.
        falling=ids[onset:minimum+1]
        metrics=dict(count=count,bullet_mps=float(rows[index]['bullet_speed_mps']),
                     target_rpm=target,base_rpm=base.tolist(),
                     minimum_rpm=rpm[ids].min(axis=0).tolist(),
                     onset_relative_ms=onset_each,minimum_relative_ms=minimum_each,
                     falling_duration_ms=round((times[ids[minimum]]-origin)*1000,2),
                     message_after_minimum_ms=round((received-times[ids[minimum]])*1000,2),
                     command_peak_during_fall_raw=command[falling].max(axis=0).tolist(),
                     current_peak_during_fall_raw=current[falling].max(axis=0).tolist(),
                     dial_speed_at_onset_dps=float(rows[ids[onset]]['dial_speed_dps']))
        results.append(metrics)
        plot_ids=np.flatnonzero((times>=origin-.012)&(times<=origin+.08)&(segment==segment[index]))
        x=(times[plot_ids]-origin)*1000
        response=plot_ids[times[plot_ids]>=origin]
        metrics['command_peak_after_drop_raw']=command[response].max(axis=0).tolist()
        metrics['command_peak_relative_ms']=[round((times[response[int(np.argmax(command[response,w]))]]-origin)*1000,2) for w in range(3)]
        metrics['current_peak_relative_ms']=[round((times[response[int(np.argmax(current[response,w]))]]-origin)*1000,2) for w in range(3)]
        for wheel in range(3):
            for row,data in enumerate((rpm,command,current)):
                axes[row,wheel].plot(x,data[plot_ids,wheel],label=f'#{count} {metrics["bullet_mps"]:.3f}',alpha=.8)
    if not results:
        raise ValueError('No isolated speed-drop events found for selected counts')
    for wheel,title in enumerate(('12 oclock','4 oclock','8 oclock')):
        axes[0,wheel].set_title(title)
        for row,label in enumerate(('RPM','Command (raw)','Current feedback (raw)')):
            axes[row,wheel].set_ylabel(label)
            axes[row,wheel].grid(alpha=.25)
            axes[row,wheel].axvline(0,color='black',lw=.7,ls='--')
        axes[2,wheel].set_xlabel('ms from detected mean-RPM drop')
    axes[0,2].legend(fontsize=7)
    figures.suptitle('Three-wheel drop alignment; onset = mean RPM below baseline by 100 RPM\nTelemetry timing is not a physical contact measurement')
    figures.tight_layout(rect=(0,0,1,.94))
    output.mkdir(parents=True,exist_ok=True)
    figures.savefig(output/'wheel_contact.png',dpi=150)
    plt.close(figures)
    report=dict(session=str(session.resolve()),counts=[first,last],
                median_sample_interval_ms=float(np.median(delta[(delta>0)&~breaks])),
                interpretation='Speed-drop thresholds are telemetry markers, not contact sensors. Raw current units. No causal fit from this small sample.',shots=results)
    (output/'wheel_contact.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    parser.add_argument('--first',type=int,required=True)
    parser.add_argument('--last',type=int,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    report=analyze(args.session,args.first,args.last,args.out)
    print(json.dumps(report,ensure_ascii=False,indent=2))
