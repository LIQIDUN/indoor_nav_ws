import unittest
from indoor_nav.publication import PublicationTracker


class PublicationAccounting(unittest.TestCase):
    def test_stop_reason_change_recovery_and_last_success(self):
        t=PublicationTracker()
        status,events=t.update(1.,True,'OK',{'imu_age_s':.01})
        self.assertEqual(status['last_success_s'],1.)
        self.assertEqual(events,[])
        status,events=t.update(1.5,False,'FC_IMU_STALE',{'imu_age_s':.5})
        self.assertEqual(events[0]['event'],'STOPPED')
        self.assertEqual(events[0]['last_success_s'],1.)
        self.assertEqual(events[0]['imu_age_s'],.5)
        self.assertEqual(t.update(2.,False,'FC_IMU_STALE',{})[1],[])
        status,events=t.update(2.5,False,'FC_LOST',{})
        self.assertEqual(events[0]['event'],'REASON_CHANGED')
        self.assertEqual(status['stopped_since_s'],1.5)
        status,events=t.update(3.,True,'OK',{})
        self.assertEqual(events[0]['event'],'RESUMED')
        self.assertEqual(events[0]['duration_s'],1.5)
        self.assertEqual(events[0]['last_success_before_gap_s'],1.)
        self.assertEqual(status['last_success_s'],3.)
    def test_clock_rewind_does_not_report_negative_outage(self):
        t=PublicationTracker();t.update(10.,False,'FC_LOST',{})
        status,events=t.update(1.,False,'FC_LOST',{})
        self.assertEqual(events[0]['event'],'TIME_RESET')
        self.assertEqual(status['outage_duration_s'],0.)

    def test_callback_gap_reported_without_faking_a_health_failure(self):
        t=PublicationTracker(expected_period=1./30.)
        t.update(1.,True,'OK',{})
        status,events=t.update(2.,True,'OK',{})
        self.assertTrue(status['sent_this_tick'])
        self.assertEqual(events[0]['event'],'TICK_GAP')
        self.assertEqual(events[0]['last_success_s'],1.)
        self.assertAlmostEqual(events[0]['estimated_missing_since_s'],1.+1./30.)
