#!/usr/bin/env python3
"""Offline comparison plot from two audit JSONL(.gz) captures; no ROS needed."""
import argparse
import gzip
import json
from collections import defaultdict


def read(path):
    opener=gzip.open if str(path).endswith('.gz') else open
    data=defaultdict(list)
    with opener(path,'rt') as source:
        for line in source:
            row=json.loads(line)
            topic=row.get('topic')
            if topic in ('/mavros/odometry/out','/mavros/imu/data'):
                data[topic].append(row)
    return data


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('before');p.add_argument('after');p.add_argument('--output',required=True)
    args=p.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(11,6),constrained_layout=True)
    for column,(path,label) in enumerate(((args.before,'Before: 60 s'),(args.after,'After: 600 s'))):
        data=read(path);rows=data['/mavros/odometry/out']
        ts=[r['elapsed_s'] for r in rows]
        axes[0,column].plot(ts[1:],[b-a for a,b in zip(ts,ts[1:])],lw=.6)
        axes[0,column].set_yscale('log');axes[0,column].set_ylim(.01,10)
        axes[0,column].axhline(1./30.,ls='--',color='green',label='30 Hz period')
        axes[0,column].set_title(label);axes[0,column].set_ylabel('ExternalNav inter-arrival (s)')
        rows=data['/mavros/imu/data']
        axes[1,column].plot([r['elapsed_s'] for r in rows],[r['header_age_s']*1000 for r in rows],lw=.6)
        axes[1,column].axhline(0,ls='--',color='red')
        axes[1,column].set_ylim(-10,10);axes[1,column].set_ylabel('IMU receipt - header (ms)')
        axes[1,column].set_xlabel('Capture elapsed (s)')
        for axis in axes[:,column]:axis.grid(alpha=.25)
    fig.suptitle('Real disarmed SITL captures; negative header age means a future timestamp')
    fig.savefig(args.output,dpi=140)

if __name__=='__main__':main()
