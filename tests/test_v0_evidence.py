import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('v0_altitude_reader', ROOT / 'tools/v0_dataflash_altitude.py')
reader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reader)


def format_packet(kind, length, name, fmt, columns):
    return b'\xa3\x95\x80' + struct.pack('<BB4s16s64s', kind, length, name.encode(), fmt.encode(), columns.encode())


class AltitudeEvidence(unittest.TestCase):
    def test_fmt_units_and_home_origin_distinction(self):
        payload = (format_packet(1, 24, 'ORGN', 'QBLLe', 'TimeUS,Type,Lat,Lng,Alt')
                   + format_packet(2, 19, 'CTUN', 'Qff', 'TimeUS,Alt,DAlt')
                   + b'\xa3\x95\x02' + struct.pack('<Qff', 100, .86, .87)
                   + b'\xa3\x95\x01' + struct.pack('<Q B i i i', 101, 0, 0, 0, 58399)
                   + b'\xa3\x95\x01' + struct.pack('<Q B i i i', 102, 1, 0, 0, 58386))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'test.BIN'
            path.write_bytes(payload)
            result = reader.altitude_evidence(path)
        self.assertEqual(len(result['events']), 2)
        origin, home = result['events']
        self.assertAlmostEqual(1 + home['data']['Alt_m'] - origin['data']['Alt_m'], .87)
        self.assertAlmostEqual(home['preceding_CTUN_m']['DAlt'], .87, places=6)

    def test_truncated_packet_never_becomes_pass_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'truncated.BIN'
            path.write_bytes(b'\xa3\x95\x80bad')
            self.assertEqual(reader.altitude_evidence(path)['events'], [])

    def test_existing_formal_results_have_no_invented_outcomes(self):
        for path in (ROOT / 'docs/v0/evidence').glob('*/summary.json'):
            with self.subTest(path=str(path)):
                result = json.loads(path.read_text())
                self.assertIn(result['result'], ('PASS', 'FAIL', 'BLOCKED'))


if __name__ == '__main__':
    unittest.main()
