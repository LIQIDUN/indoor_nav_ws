import math
import json
from pathlib import Path
import unittest
from indoor_nav.core import FreezeWindow

class FreezeTests(unittest.TestCase):
    def feed(self, position, truth=None, velocity=(.3,0.)):
        detector=FreezeWindow()
        for i in range(27):
            t=i*.04
            detector.observe(10+t, position(t), velocity, truth(t) if truth else None,
                             {'world_header_age_s':.02,'velocity_header_age_s':.03})
        return detector

    def test_dataflash_343_replay_reproduces_old_false_alarm(self):
        rows=json.loads((Path(__file__).parent/'fixtures/freeze_343_window.json').read_text())['samples']
        d=FreezeWindow()
        anchor=None; start=None; old_trigger=False
        for r in rows:
            speed=math.hypot(*r['ekf_ned_velocity_xy'])
            xy=r['visp_ned_xy']; t=r['time_s']
            if speed < .20:
                anchor=xy;start=None
            else:
                if anchor is None:anchor=xy
                if math.hypot(xy[0]-anchor[0],xy[1]-anchor[1])>.015:
                    anchor=xy;start=None
                else:
                    if start is None:start=t
                    old_trigger = old_trigger or t-start>1.
            # Only speed norm is used from EKF; position deltas are NED in both traces.
            d.observe(t,xy,(speed,0),r['sim2_ned_xy'])
        self.assertTrue(old_trigger)
        self.assertFalse(d.snapshot['triggered'])
        self.assertEqual(d.snapshot['classification'],'STATIONARY_WITH_EKF_VELOCITY_DISAGREEMENT')
        self.assertTrue(all(r['visp_position_error_m']==100 for r in rows))

    def test_frozen_uwb_with_independent_movement_triggers(self):
        d=self.feed(lambda t:(0,0),lambda t:(.3*t,0))
        self.assertTrue(d.snapshot['triggered'])
        self.assertGreaterEqual(d.snapshot['truth_span_m'],.3)
        self.assertEqual(d.snapshot['uwb_span_m'],0)
        self.assertEqual(d.snapshot['motion_source'],'GAZEBO_TRUTH')
        self.assertEqual(d.snapshot['samples'][0]['context']['world_header_age_s'],.02)

    def test_actual_flight_stationary_truth_and_drifting_ekf_is_not_freeze(self):
        # DataFlash 343: true speed .003-.014 while XKF1 speed .202-.211.
        d=self.feed(lambda t:(.006*t,0),lambda t:(.006*t,0),(.205,0))
        self.assertFalse(d.snapshot['triggered'])
        self.assertEqual(d.snapshot['classification'],'STATIONARY_WITH_EKF_VELOCITY_DISAGREEMENT')

    def test_normal_motion_and_hover(self):
        for velocity in ((0.,0.),(.3,0.)):
            d=self.feed(lambda t:(.3*t,0),lambda t:(.3*t,0),velocity)
            self.assertFalse(d.snapshot['triggered'])
        d=self.feed(lambda t:(0,0),lambda t:(0,0),(0,0))
        self.assertEqual(d.snapshot['classification'],'STATIONARY')

    def test_out_and_back_not_mistaken_for_static(self):
        d=self.feed(lambda t:(.04*math.sin(t*2*math.pi),0),lambda t:(.3*t,0))
        self.assertFalse(d.snapshot['triggered'])
        self.assertGreater(d.snapshot['uwb_span_m'],.015)

    def test_timer_reusing_sample_does_not_build_window(self):
        d=FreezeWindow()
        for i in range(100):
            self.assertFalse(d.observe(10.,(0,0),(.3,0),(.3,0)))
        self.assertEqual(len(d.samples),1)

    def test_gaps_rewind_and_source_change_clear_history(self):
        d=self.feed(lambda t:(0,0),lambda t:(.3*t,0))
        for t,truth in [(12.,(.6,0)),(9.,(.1,0)),(9.04,None)]:
            self.assertFalse(d.observe(t,(0,0),(.3,0),truth))
            self.assertEqual(len(d.samples),1)
        self.assertFalse(d.observe(9.08,(float('nan'),0),(.3,0)))
        self.assertEqual(d.snapshot['classification'],'INVALID_SAMPLE')

    def test_low_speed_resets_hardware_motion_evidence(self):
        d=self.feed(lambda t:(0,0))
        self.assertTrue(d.snapshot['triggered'])
        self.assertFalse(d.observe(11.08,(0,0),(0,0)))

    def test_frozen_gazebo_pose_with_moving_twist_is_source_anomaly(self):
        d=FreezeWindow()
        for i in range(27):
            self.assertFalse(d.observe(10+i*.04,(0,0),(.3,0),(0,0),{'truth_speed_mps':.3}))
        self.assertEqual(d.snapshot['classification'],'GAZEBO_POSE_TWIST_INCONSISTENT')

    def test_full_window_snapshot_is_detached_from_reset(self):
        d=self.feed(lambda t:(0,0),lambda t:(.3*t,0))
        snapshot=d.snapshot
        d.reset('STALE')
        self.assertTrue(snapshot['triggered'])
        self.assertGreater(len(snapshot['samples']),20)
        self.assertEqual(d.snapshot['classification'],'STALE')

if __name__=='__main__':
    unittest.main()
