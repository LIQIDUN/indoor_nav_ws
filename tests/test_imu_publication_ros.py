import gzip
import json
from pathlib import Path
import unittest
from test_mock_faults_ros import _TransportFixture
from test_pipeline_ros_messages import ROS,module
if ROS:
    import rospy
    from sensor_msgs.msg import Imu
    from mavros_msgs.msg import State
    from indoor_nav.core import Transform2D


@unittest.skipUnless(ROS,'generated ROS messages required')
class ImuPublication(_TransportFixture):
    def setUp(self):
        super().setUp()
        self.w=module('world_estimator').WorldEstimator()
        self.w.transform=Transform2D();self.w.raw_xy=(0,0)
        self.w.output_enabled=True
        self.w.fc_state=State(connected=True,armed=False)
        self.refresh()
    def refresh(self):
        self.w.fc_stamp=self.now
        self.w.height=.8;self.w.height_stamp=self.now;self.w.height_header_stamp=self.now
        self.w.uwb.last=(self.now,0.,0.)
    def imu(self,offset=0):
        m=Imu();m.orientation.w=1.
        m.header.stamp=rospy.Time.from_sec(self.now+offset)
        self.w.imu_cb(m)
        return m
    def test_small_future_header_waits_without_restamping_or_freshening(self):
        msg=self.imu(.005)
        self.assertIsNone(self.w.fc_imu)
        self.assertFalse(self.w.health(self.now)[0])
        self.now+=.004
        self.assertIsNone(self.w.fc_imu)
        self.assertFalse(self.w.health(self.now)[0])
        self.now+=.002
        self.assertTrue(self.w.health(self.now)[0])
        self.assertIs(self.w.fc_imu,msg)
        self.assertEqual(self.w.fc_imu_stamp,10.)
        self.assertAlmostEqual(msg.header.stamp.to_sec(),10.005)
        self.now+=.4;self.refresh()
        self.assertEqual(self.w.health(self.now),(False,'FC_IMU_STALE'))
    def test_large_future_stale_and_expired_pending_are_rejected(self):
        self.imu(.021);self.assertIsNone(self.w.pending_imu)
        self.assertIsNone(self.w.fc_imu)
        self.imu(-.401);self.assertIsNone(self.w.fc_imu)
        self.imu(.005);self.now+=.5;self.refresh()
        self.assertFalse(self.w.health(self.now)[0])
        self.assertIsNone(self.w.pending_imu)
        self.assertIsNone(self.w.fc_imu)
    def test_stop_and_resume_snapshots_preserve_gates(self):
        self.imu();self.w.publish(None)
        self.assertEqual(len(self.w.fc_odom_pub.messages),1)
        self.now+=.41;self.refresh();self.w.publish(None)
        self.assertEqual(len(self.w.fc_odom_pub.messages),1)
        event=json.loads(self.w.publication_events_pub.messages[-1].data)
        self.assertEqual(event['event'],'STOPPED')
        self.assertEqual(event['reason'],'FC_IMU_STALE')
        self.assertGreater(event['imu_age_s'],.4)
        self.assertEqual(event['last_success_s'],10.)
        self.now+=.01;self.refresh();self.imu();self.w.publish(None)
        event=json.loads(self.w.publication_events_pub.messages[-1].data)
        self.assertEqual(event['event'],'RESUMED')
        self.assertEqual(len(self.w.fc_odom_pub.messages),2)
        self.w.fc_state.connected=False;self.w.publish(None)
        self.assertEqual(len(self.w.fc_odom_pub.messages),2)
        self.assertEqual(json.loads(self.w.publication_events_pub.messages[-1].data)['reason'],'FC_LOST')
    def test_transport_stop_distinct_from_healthy_world(self):
        self.imu();self.w.transport_adapter.enabled=True
        self.w.publish(None)
        self.assertEqual(len(self.w.world_pub.messages),1)
        self.assertEqual(len(self.w.fc_odom_pub.messages),0)
        d=json.loads(self.w.diagnostics_pub.messages[-1].data)
        self.assertTrue(d['healthy']);self.assertTrue(d['world_publish_enabled'])
        self.assertTrue(d['publication']['external_nav']['reason'].startswith('TRANSPORT_ADAPTER:'))

    def test_recorded_baseline_timing_replay(self):
        # Replay REAL receipt/header and publish timing only; identity attitude and
        # fresh UWB/range are synthetic controls, not an online fusion result.
        path=Path(__file__).resolve().parents[1]/'docs/evidence/mavlink_baseline.jsonl.gz'
        with gzip.open(path,'rt') as source:
            rows=[json.loads(line) for line in source]
        rows=sorted((r for r in rows if r.get('topic') in
                    ('/mavros/imu/data','/indoor_nav/localization_diagnostics')),key=lambda r:r['ros_s'])
        old_receipt=None;old_header=None;old_stale=0;new_stale=0;total=0
        first=rows[0]['ros_s']
        for row in rows:
            self.now=row['ros_s'];self.refresh()
            if row['topic']=='/mavros/imu/data':
                self.imu(row['header_s']-self.now)
                if 0<=self.now-row['header_s']<=.4:
                    old_receipt=self.now;old_header=row['header_s']
            elif self.now-first>1.:
                total+=1
                old_stale+=int(old_receipt is None or not 0<=self.now-old_receipt<=.4 or not 0<=self.now-old_header<=.4)
                new_stale+=int(not self.w.health(self.now)[0])
        self.assertGreater(total,1000)
        self.assertGreater(old_stale,total//2)
        self.assertEqual(new_stale,0)
