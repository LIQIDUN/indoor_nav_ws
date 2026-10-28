#!/usr/bin/env python3
"""Create an additive run index; never change historical results or raw logs."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'docs/v0/evidence'


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def inventory(paths):
    return [{'path': str(path.relative_to(ROOT)), 'bytes': path.stat().st_size, 'sha256': digest(path),
             'local_binary': path.suffix in ('.bag', '.active', '.BIN', '.bin', '.tlog', '.raw')}
            for path in sorted(set(paths)) if path.is_file() and not path.is_symlink()]


def read(path, default=None):
    return json.loads(path.read_text()) if path.exists() else default


def suite_outcome(results, incomplete):
    if incomplete or any(row['test_exit_code'] != 0 or not row.get('ports_released') for row in results):
        return 'FAIL'
    return 'PASS' if results else 'INCOMPLETE'


def recordings(directory):
    paths = sorted(directory.resolve().rglob('*.bag'))
    if not paths:
        return []
    import rosbag
    result = []
    for path in paths:
        row = {'path': str(path.relative_to(ROOT))}
        try:
            with rosbag.Bag(str(path)) as bag:
                row.update(readable=True, start_ros_s=bag.get_start_time(), end_ros_s=bag.get_end_time(),
                           topics={name: dict(messages=info.message_count, type=info.msg_type)
                                   for name, info in bag.get_type_and_topic_info().topics.items()})
                states = [dict(ros_s=stamp.to_sec(), armed=message.armed, connected=message.connected, mode=message.mode)
                          for _, message, stamp in bag.read_messages(topics=['/mavros/state'])]
                row['first_state'] = states[0] if states else None
                row['last_state'] = states[-1] if states else None
                sequences = {}
                for topic, message, _ in bag.read_messages(topics=['/sound_sensor/audio_raw', '/motor_mic/audio_raw', '/vibration_data']):
                    if message._type == 'std_msgs/UInt8MultiArray':
                        sequences.setdefault(topic, []).append(int.from_bytes(bytes(message.data[:8]), 'little'))
                row['synthetic_sequences'] = {topic: dict(messages=len(sequence),
                    missing_inside_epochs=sum(max(0, later-earlier-1) for earlier, later in zip(sequence, sequence[1:]) if later > earlier),
                    resets=sum(later <= earlier for earlier, later in zip(sequence, sequence[1:]))) for topic, sequence in sequences.items()}
        except Exception as error:
            row.update(readable=False, error=str(error))
        result.append(row)
    return result


def archive(prefix, output, support=()):
    if output.parent.resolve() != EVIDENCE.resolve():
        raise ValueError('output must be a new directory directly under evidence')
    output.mkdir(exist_ok=False)
    sources = ('src/indoor_nav/src/indoor_nav/v0.py', 'src/indoor_nav/scripts/relative_mission_manager.py',
               'src/indoor_nav/scripts/v0_nav_health.py')
    current = {name: digest(ROOT / name) for name in sources}
    runs, old, files, runtimes = [], [], [], set()
    for directory in support:
        directory = directory.resolve()
        if directory.parent != EVIDENCE.resolve():
            raise ValueError('support directory must be directly under evidence')
        files.extend(directory.rglob('*'))
    for summary_path in sorted(EVIDENCE.glob('*/summary.json')):
        summary = read(summary_path)
        if not summary_path.parent.name.startswith(prefix):
            old.append({'path': str(summary_path.relative_to(ROOT)), 'result': summary['result'],
                        'sha256': digest(summary_path)})
            continue
        directory = summary_path.parent
        inputs = read(directory / 'inputs.json', {})
        provenance = []
        for path in sorted(directory.glob('navigation/*/provenance.json')):
            data = read(path)
            provenance.append({'path': str(path.relative_to(ROOT)),
                               'production_hash_matches': data.get('code_sha256') == current,
                               'yaml_hash_matches': digest(path.parent / 'loaded.yaml') == data.get('yaml_sha256')})
        runs.append({'path': str(summary_path.relative_to(ROOT)), 'case': summary.get('case'),
                     'result': summary['result'], 'error': summary.get('error'), 'inputs': inputs,
                     'cleanup': read(directory / 'cleanup.json'), 'provenance': provenance,
                     'recordings': recordings(directory),
                     'bag_analysis': str((directory / 'bag_analysis.json').relative_to(ROOT)) if (directory / 'bag_analysis.json').exists() else None})
        files.extend(directory.rglob('*'))
        if inputs.get('runtime'):
            runtimes.add(Path(inputs['runtime']).resolve())
    suites = []
    for directory in sorted(EVIDENCE.glob('resume-suite-' + prefix + '*')):
        incomplete = read(directory / 'incomplete.json')
        results = read(directory / 'results.json', [])
        status = suite_outcome(results, incomplete)
        suites.append({'path': str(directory.relative_to(ROOT)), 'result': status,
                       'completed_results': results, 'incomplete': incomplete})
        files.extend(directory.rglob('*'))
    for runtime in sorted(runtimes):
        if runtime.parent != ROOT / '.sitl_runtime/v0':
            raise ValueError('unexpected runtime path: ' + str(runtime))
        files.extend(runtime.rglob('*'))
        destination = output / 'runtime_logs' / runtime.name
        destination.mkdir(parents=True)
        for name in ('inputs.sha256', 'ardupilot_commit.txt', 'gui.txt', 'arducopter.log', 'gazebo.log', 'mavros.log', 'mavproxy.log', 'roscore.log'):
            path = runtime / name
            if path.exists():
                shutil.copyfile(path, destination / (name + '.txt' if path.suffix == '.log' else name))
    matrix = {'generated_wall_s': time.time(), 'archive_git_commit': subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
              'current_production_sha256': current, 'runs': runs, 'suites': suites,
              'analysis_tools_sha256': {name: digest(ROOT / name) for name in ('tools/v0_analyze_bag.py', 'tools/v0_archive_dynamic_evidence.py') if (ROOT / name).exists()},
              'note': 'flight outcome and suite shutdown outcome are distinct; historical FAIL stays FAIL; hashes do not certify physical safety',
              'F2': 'CANCELLED_BY_USER', 'physical_acquisition': 'BLOCKED_HARDWARE_NOT_VERIFIED', 'real_aircraft': 'NOT_AUTHORIZED_NOT_RUN'}
    (output / 'run_matrix.json').write_text(json.dumps(matrix, ensure_ascii=False, indent=2))
    (output / 'historical_index.json').write_text(json.dumps(old, indent=2))
    manifest = {'self_excluded': True, 'files': inventory(files + list(output.rglob('*')))}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    print(json.dumps({'output': str(output), 'new_runs': len(runs), 'historical_runs': len(old), 'suites': len(suites), 'files': len(manifest['files'])}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--support', nargs='*', type=Path, default=[])
    args = parser.parse_args()
    archive(args.prefix, args.output.resolve(), args.support)
