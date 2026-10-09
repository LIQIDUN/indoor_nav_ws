#!/usr/bin/env python3
"""Offline shadow evaluation. CSV: t,x,y,truth_vx,truth_vy (accepted body XY).
Truth columns are evaluation only; never passed into the estimator.
"""
import argparse
import csv
import json
import math
import random
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/indoor_nav/src'))
from indoor_nav.velocity import VelocityWindow


def evaluate(rows):
    fit=VelocityWindow()
    errors=[];delays=[];reasons={};samples=[]
    for row in rows:
        t,x,y,vx,vy=map(float,row)
        fit.add(t,(x,y));d=fit.estimate(t)
        reasons[d['reason']]=reasons.get(d['reason'],0)+1
        if d['valid']:
            error=math.hypot(d['velocity_world_xy'][0]-vx,d['velocity_world_xy'][1]-vy)
            errors.append(error);delays.append(d['delay_s'])
            samples.append({'t':t,'estimate':d['velocity_world_xy'],'truth':[vx,vy],
                            'error_mps':error,'sigma_mps':math.sqrt(max(d['variance_world_xy']))})
    return {'samples':len(rows),'valid':len(errors),'reasons':reasons,
            'rmse_vector_mps':math.sqrt(sum(e*e for e in errors)/len(errors)) if errors else None,
            'max_error_mps':max(errors) if errors else None,
            'mean_window_delay_s':sum(delays)/len(delays) if delays else None,
            'comparison':'truth at newest sample; acceleration includes smoothing lag',
            'trace':samples}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv',type=Path)
    args=parser.parse_args()
    if args.csv:
        with args.csv.open() as f:
            rows=[[r[k] for k in ('t','x','y','truth_vx','truth_vy')] for r in csv.DictReader(f)]
        result={'provenance':str(args.csv),'result':evaluate(rows)}
    else:
        rng=random.Random(457);cases={}
        for case in ('static','linear','accelerating','outliers','frozen'):
            rows=[]
            for i in range(250):
                t=i*.04;vx=.3;vy=-.2;x=vx*t;y=vy*t
                if case=='static':x=y=vx=vy=0.
                if case=='accelerating':x=.04*t*t;vx=.08*t
                if case=='frozen' and i>=125:x=.3*4.96;y=-.2*4.96
                noise=0. if case=='frozen' else .01
                x+=rng.gauss(0,noise);y+=rng.gauss(0,noise)
                if case=='outliers' and i%37==0:x+=.2
                rows.append((t,x,y,vx,vy))
            cases[case]=evaluate(rows)
        result={'provenance':'SYNTHETIC seed457 25Hz sigmaXY=.01m; NOT hardware accuracy',
                'configuration':'production default sigma floor .10m, window .50s', 'cases':cases}
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
