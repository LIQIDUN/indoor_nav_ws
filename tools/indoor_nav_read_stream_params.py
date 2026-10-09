#!/usr/bin/env python3
"""Read fresh SITL stream parameters with PARAM_REQUEST_READ, not MAVROS cache.
Only emits message ID20. No PARAM_SET, stream requests, commands or mode changes.
"""
import json
import struct
import threading
import time
import rospy
from mavros_msgs.msg import Mavlink,State
from mavros.mavlink import convert_to_rosmsg
from pymavlink import mavutil

PARAMS=('SIM_SPEEDUP','SERIAL0_PROTOCOL','SR0_RAW_SENS','SR0_EXTRA1','SR0_POSITION',
        'SR1_RAW_SENS','SR1_EXTRA1','SR1_POSITION','EK3_SRC1_VELXY','EK3_SRC1_POSXY','EK3_SRC1_POSZ')


def main():
    rospy.init_node('indoor_nav_read_stream_params',anonymous=True)
    assert rospy.get_param('/indoor_nav/safety/profile',None)=='sitl','SITL profile required'
    state=rospy.wait_for_message('/mavros/state',State,timeout=5)
    assert state.connected and not state.armed,'Connected disarmed SITL required'
    condition=threading.Condition();values={}
    def receive(msg):
        if msg.msgid!=22 or msg.sysid!=1 or msg.compid!=1 or msg.framing_status!=Mavlink.FRAMING_OK:return
        payload=struct.pack('<{}Q'.format(len(msg.payload64)),*msg.payload64)
        value,count,index,name,kind=struct.unpack('<fHH16sB',payload[:25])
        name=name.rstrip(b'\0').decode('ascii')
        if name not in PARAMS:return
        with condition:
            values[name]={'value':value,'param_type':kind,'receipt_wall_s':time.time(),'index':index}
            condition.notify_all()
    sub=rospy.Subscriber('/mavlink/from',Mavlink,receive,queue_size=100)
    pub=rospy.Publisher('/mavlink/to',Mavlink,queue_size=10,latch=True)
    deadline=time.monotonic()+3
    while pub.get_num_connections()==0 and time.monotonic()<deadline:time.sleep(.05)
    assert pub.get_num_connections()>0,'MAVROS raw bridge unavailable'
    mav=mavutil.mavlink.MAVLink(None,srcSystem=246,srcComponent=191)
    for name in PARAMS:
        message=mavutil.mavlink.MAVLink_param_request_read_message(1,1,name.encode('ascii'),-1)
        message.pack(mav)
        mav.seq=(mav.seq+1)%256
        assert message.get_msgId()==20
        with condition:
            values.pop(name,None)
            for attempt in range(3):
                pub.publish(convert_to_rosmsg(message))
                deadline=time.monotonic()+2
                while name not in values and time.monotonic()<deadline:
                    condition.wait(max(0.,deadline-time.monotonic()))
                if name in values:break
        assert name in values,'No fresh response for '+name
        if name=='SIM_SPEEDUP':assert values[name]['value']>0,'SITL not confirmed'
    print(json.dumps({'source':'fresh FC PARAM_VALUE replies to PARAM_REQUEST_READ (ID20)',
                      'wall_s':time.time(),'state':{'connected':state.connected,'armed':state.armed},'params':values},indent=2))
    sub.unregister()

if __name__=='__main__':main()
