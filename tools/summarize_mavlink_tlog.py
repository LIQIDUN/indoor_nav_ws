#!/usr/bin/env python3
"""Offline extract stream ownership and ExternalNav forwarding from MAVProxy tlog.
Limits are wall-clock epochs matching the synchronized ROS audit. Reads only.
"""
import argparse
import collections
import json
from pymavlink import mavutil


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('tlog');p.add_argument('--start',type=float,default=0.);p.add_argument('--end',type=float,default=float('inf'))
    args=p.parse_args();link=mavutil.mavlink_connection(args.tlog)
    intervals=[];stream_requests=[];odom_times=[];commands=collections.Counter();sets=[];reads=[];boot_clocks={}
    while True:
        msg=link.recv_match(type=['REQUEST_DATA_STREAM','COMMAND_LONG','ODOMETRY','PARAM_SET','PARAM_REQUEST_READ','ATTITUDE','LOCAL_POSITION_NED'])
        if msg is None:break
        stamp=msg._timestamp
        if not args.start<=stamp<=args.end:continue
        src=[msg.get_srcSystem(),msg.get_srcComponent()]
        if msg.get_type() in ('ATTITUDE','LOCAL_POSITION_NED'):
            row={'wall_s':stamp,'boot_s':msg.time_boot_ms/1000.}
            clock=boot_clocks.setdefault(msg.get_type(),{'first':row,'last':row,'rewinds':0})
            if row['boot_s']<clock['last']['boot_s']:clock['rewinds']+=1
            clock['last']=row
        elif msg.get_type()=='REQUEST_DATA_STREAM':
            stream_requests.append({'wall_s':stamp,'source':src,'stream_id':msg.req_stream_id,'rate_hz':msg.req_message_rate,'on':msg.start_stop})
        elif msg.get_type()=='COMMAND_LONG':
            commands[str(msg.command)]+=1
            if msg.command==511:intervals.append({'wall_s':stamp,'source':src,'message_id':msg.param1,'interval_us':msg.param2})
        elif msg.get_type()=='ODOMETRY':odom_times.append(stamp)
        elif msg.get_type()=='PARAM_SET':sets.append({'wall_s':stamp,'source':src,'name':msg.param_id,'value':msg.param_value})
        elif msg.get_type()=='PARAM_REQUEST_READ':reads.append({'wall_s':stamp,'source':src,'name':msg.param_id})
    odom={'count':len(odom_times),'observed_hz':(len(odom_times)-1)/(odom_times[-1]-odom_times[0]) if len(odom_times)>1 else None,
          'max_gap_s':max((b-a for a,b in zip(odom_times,odom_times[1:])),default=None),
          'claim':'MAVProxy forwarded to FC TCP socket; not proof EKF accepted/fused'}
    print(json.dumps({'start_wall_s':args.start,'end_wall_s':args.end,'stream_requests':stream_requests,
                      'message_interval_requests':intervals,'command_counts':commands,'parameter_sets':sets,
                      'parameter_reads':reads,'fc_boot_clocks':boot_clocks,'odometry':odom},indent=2))

if __name__=='__main__':main()
