#!/usr/bin/env python3
"""Offline index and byte manifest; preserves run outcomes and ignored binaries."""
import hashlib
import json
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'docs/v0/evidence'


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b''):
            result.update(block)
    return result.hexdigest()


def main():
    sources = ('src/indoor_nav/src/indoor_nav/v0.py', 'src/indoor_nav/scripts/relative_mission_manager.py',
               'src/indoor_nav/scripts/v0_nav_health.py')
    current = {name: digest(ROOT / name) for name in sources}
    rows, runtimes = [], set()
    for summary_path in sorted(EVIDENCE.glob('*/summary.json')):
        directory = summary_path.parent
        summary = json.loads(summary_path.read_text())
        inputs = json.loads((directory / 'inputs.json').read_text()) if (directory / 'inputs.json').exists() else {}
        provenance = []
        for path in sorted(directory.glob('navigation/*/provenance.json')):
            data = json.loads(path.read_text())
            provenance.append({'path': str(path.relative_to(ROOT)), **data,
                               'loaded_yaml_hash_valid': digest(path.parent / 'loaded.yaml') == data.get('yaml_sha256'),
                               'matches_current_production': data.get('code_sha256') == current})
        runtime = inputs.get('runtime') or summary.get('identity', {}).get('runtime')
        if runtime:
            runtimes.add(runtime)
        rows.append({'path': str(summary_path.relative_to(ROOT)), 'case': summary.get('case'),
                     'result': summary['result'], 'error': summary.get('error'),
                     'start_wall_s': summary.get('start_wall_s'), 'end_wall_s': summary.get('end_wall_s'),
                     'inputs': inputs, 'provenance': provenance,
                     'cleanup': json.loads((directory / 'cleanup.json').read_text()) if (directory / 'cleanup.json').exists() else None})
    required = ('single', 'multi', 'yaw', 'altitude', 'distance_short', 'distance_long', 'finish_stop', 'abort',
                'manager_kill', 'manager_exit', 'manager_pause', 'pose_loss', 'mavros_loss', 'mode_exit',
                'gate_pause', 'gate_kill', 'publisher_conflict', 'concurrent', 'f3', 'f3_auto_bag', 'link_fault', 'dry_run')
    canonical = {}
    for case in required:
        candidates = [row for row in rows if row['case'] == case and any(item['matches_current_production'] for item in row['provenance'])]
        latest = max(candidates, key=lambda row: row.get('start_wall_s') or 0) if candidates else None
        canonical[case] = {'result': latest['result'] if latest else 'NOT_RUN', 'path': latest['path'] if latest else None}
    (EVIDENCE / 'resume_acceptance.json').write_text(json.dumps({'current_production_sha256': current,
        'latest_matching_production_cases': canonical, 'all_required_latest_cases_pass': all(item['result'] == 'PASS' for item in canonical.values()),
        'note': 'latest result, not latest PASS; real hardware is not certified; source hashes do not certify unchanged test tooling'}, indent=2))
    suites = []
    for directory in sorted(EVIDENCE.glob('resume-suite-*')):
        if (directory / 'incomplete.json').exists():
            suites.append({'path': str(directory.relative_to(ROOT)), 'result': 'FAIL',
                           'details': json.loads((directory / 'incomplete.json').read_text())})
        elif (directory / 'results.json').exists():
            details = json.loads((directory / 'results.json').read_text())
            suites.append({'path': str(directory.relative_to(ROOT)),
                           'result': 'PASS' if all(row['test_exit_code'] == 0 and row.get('ports_released') for row in details) else 'FAIL',
                           'details': details})
        else:
            suites.append({'path': str(directory.relative_to(ROOT)), 'result': 'INCOMPLETE'})
    matrix = {'generated_wall_s': time.time(), 'index_git_commit': subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
              'current_production_sha256': current, 'runs': rows,
              'suites_including_startup_and_shutdown_failures': suites,
              'F2': {'status': 'CANCELLED', 'reason': 'user scope: built-in GPS SITL; no equivalent injection requirement'},
              'real_acquisition': {'status': 'BLOCKED', 'reason': 'onboard current driver/launch and physical hardware not verified'},
              'real_aircraft': {'status': 'NOT_RUN', 'reason': 'not authorized in this development session'}}
    (EVIDENCE / 'test_matrix.json').write_text(json.dumps(matrix, ensure_ascii=False, indent=2))
    runtime_rows = []
    for name in sorted(runtimes):
        runtime = Path(name).resolve()
        if runtime.parent != ROOT / '.sitl_runtime/v0' or not runtime.is_dir():
            continue
        files = []
        for path in sorted(runtime.rglob('*')):
            if path.is_file() and not path.is_symlink():
                files.append({'path': str(path.relative_to(ROOT)), 'bytes': path.stat().st_size, 'sha256': digest(path)})
        runtime_rows.append({'runtime': str(runtime.relative_to(ROOT)), 'files': files,
                             'note': 'shared instance logs may span multiple runs; correlate wall windows, not whole-log attribution'})
    (EVIDENCE / 'resume_runtime_index.json').write_text(json.dumps(runtime_rows, indent=2))
    excluded = EVIDENCE / 'manifest.json'
    entries = []
    for path in sorted(EVIDENCE.rglob('*')):
        if path == excluded or not path.is_file() or path.is_symlink():
            continue
        entries.append({'path': str(path.relative_to(ROOT)), 'bytes': path.stat().st_size, 'sha256': digest(path),
                        'local_binary': path.suffix in ('.bag', '.active', '.BIN', '.tlog', '.raw')})
    excluded.write_text(json.dumps({'generated_wall_s': time.time(), 'self_excluded': True, 'files': entries}, indent=2))
    print(json.dumps({'runs': len(rows), 'files': len(entries), 'runtimes': len(runtime_rows)}, indent=2))


if __name__ == '__main__':
    main()
