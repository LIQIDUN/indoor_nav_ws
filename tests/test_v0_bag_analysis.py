import importlib.util
from pathlib import Path
import unittest

from test_pipeline_ros_messages import ROS

if ROS:
    spec = importlib.util.spec_from_file_location('v0_bag_analysis', Path(__file__).resolve().parents[1] / 'tools/v0_analyze_bag.py')
    analyzer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(analyzer)


@unittest.skipUnless(ROS, 'ROS bag environment required')
class ReceptionAnalysis(unittest.TestCase):
    def test_raw_rewind_is_distinct_from_receipt_gap(self):
        rows = [dict(stream='pose', mono_rx_s=1., ros_rx_s=101., header_s=100.99, frame='map', interval_s=None),
                dict(stream='pose', mono_rx_s=1.04, ros_rx_s=101.04, header_s=100.98, frame='map', interval_s=.04)]
        result = analyzer.reception_statistics(rows)['pose']
        self.assertEqual(result['raw_header_rewinds'], 1)
        self.assertAlmostEqual(result['rate_hz'], 25.)
        self.assertEqual(result['callback_interval_s']['max'], .04)
        self.assertEqual(result['frames'], ['map'])

    def test_empty_and_single_message_streams(self):
        self.assertEqual(analyzer.reception_statistics([]), {})
        row = dict(stream='state', mono_rx_s=1., ros_rx_s=101., header_s=100., frame='', interval_s=None)
        self.assertEqual(analyzer.reception_statistics([row])['state']['rate_hz'], 0.)
