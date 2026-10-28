import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location('v0_dynamic_archive', Path(__file__).resolve().parents[1] / 'tools/v0_archive_dynamic_evidence.py')
archive = importlib.util.module_from_spec(spec)
spec.loader.exec_module(archive)


class DynamicArchive(unittest.TestCase):
    def test_additive_index_preserves_failed_result_and_rejects_overwrite(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            evidence = root / 'docs/v0/evidence'
            for name in ('src/indoor_nav/src/indoor_nav/v0.py', 'src/indoor_nav/scripts/relative_mission_manager.py',
                         'src/indoor_nav/scripts/v0_nav_health.py'):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text('frozen')
            run = evidence / 'test-run'
            run.mkdir(parents=True)
            summary = run / 'summary.json'
            summary.write_text(json.dumps(dict(case='yaw', result='FAIL', error='real receive gap')))
            before = summary.read_bytes()
            support = evidence / 'support'
            support.mkdir()
            (support / 'test.txt').write_text('offline result')
            output = evidence / 'archive'
            with patch.object(archive, 'ROOT', root), patch.object(archive, 'EVIDENCE', evidence), patch.object(archive.subprocess, 'check_output', return_value='commit\n'):
                archive.archive('test-', output, [Path(os.path.relpath(support))])
                self.assertEqual(json.loads((output / 'run_matrix.json').read_text())['runs'][0]['result'], 'FAIL')
                self.assertEqual(summary.read_bytes(), before)
                manifest = json.loads((output / 'manifest.json').read_text())
                self.assertIn('docs/v0/evidence/support/test.txt', [entry['path'] for entry in manifest['files']])
                with self.assertRaises(FileExistsError):
                    archive.archive('test-', output)

    def test_output_cannot_be_outside_evidence(self):
        with self.assertRaises(ValueError):
            archive.archive('test-', Path('/tmp/not-evidence/output'))

    def test_completed_failed_suite_is_fail_not_incomplete(self):
        self.assertEqual(archive.suite_outcome([dict(test_exit_code=1, ports_released=True)], None), 'FAIL')
        self.assertEqual(archive.suite_outcome([dict(test_exit_code=0, ports_released=True)], None), 'PASS')
        self.assertEqual(archive.suite_outcome([], None), 'INCOMPLETE')
        self.assertEqual(archive.suite_outcome([], dict(error='cleanup')), 'FAIL')
