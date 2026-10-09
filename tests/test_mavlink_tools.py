import importlib.util
import pathlib
import subprocess
import unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('mavlink_audit',ROOT/'tools/indoor_nav_mavlink_audit.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


class StreamTools(unittest.TestCase):
    def test_frequency_gaps_and_censored_edges(self):
        d=audit.summarize([1.,1.04,1.08],0.,2.)
        self.assertAlmostEqual(d['observed_hz'],25.)
        self.assertAlmostEqual(d['max_internal_gap_s'],.04)
        self.assertAlmostEqual(d['end_silence_s'],.92)
        d=audit.summarize([],0.,2.)
        self.assertIsNone(d['observed_hz']);self.assertEqual(d['end_silence_s'],2.)
    def test_sitl_defaults_and_explicit_rollback_without_launching(self):
        envfile=str(ROOT/'tools/sitl.env')
        for override,expected in [('', '25'),('4','4'),('30','30')]:
            command='unset SITL_MAVLINK_STREAMRATE; source "$1"; printf "%s" "$SITL_MAVLINK_STREAMRATE"'
            if override:command='export SITL_MAVLINK_STREAMRATE="$2"; source "$1"; printf "%s" "$SITL_MAVLINK_STREAMRATE"'
            result=subprocess.run(['bash','-c',command,'check',envfile,override],text=True,capture_output=True,check=True)
            self.assertEqual(result.stdout,expected)
        for filename in ('start_indoor_sitl.sh','sitl_component.sh'):
            source=(ROOT/'tools'/filename).read_text()
            self.assertIn('--mavproxy-args=',source)
            self.assertLess(source.index('set +u'),source.index('source /opt/ros/noetic/setup.bash'))
            if filename=='start_indoor_sitl.sh':
                self.assertLess(source.index('SITL_MAVLINK_STREAMRATE must'),source.index('先清理旧进程'))
            self.assertIn('--streamrate=$SITL_MAVLINK_STREAMRATE',source)
            subprocess.run(['bash','-n',str(ROOT/'tools'/filename)],check=True)
