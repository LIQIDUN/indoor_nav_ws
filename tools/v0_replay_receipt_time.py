#!/usr/bin/env python3
"""Read-only bag comparison of legacy header gates and receipt-time pose checks."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import types


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/indoor_nav/src'))
from indoor_nav.v0 import Health, load_config


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def replay(directory, baseline):
    import rosbag

    baseline_config = baseline.load_config((directory / 'mission.yaml').read_text())
    current_config = load_config((directory / 'mission.yaml').read_text())
    old, current = baseline.Health(baseline_config), Health(current_config)
    header_diagnostics = Counter()
    frames = Counter()
    gap_counts = Counter()
    previous_rx = previous_stamp = None
    largest_gap = largest_active_gap = 0.
    first_legacy_fault = first_current_fault = None
    rewinds = []
    source_offsets = []
    task_state = None
    baseline_time = None
    historical = json.loads((directory / 'summary.json').read_text())
    with rosbag.Bag(str(directory / 'flight.bag')) as bag:
        for topic, message, recorded in bag.read_messages(topics=[
                '/mavros/state', '/mavros/local_position/pose', '/v0_nav/status']):
            ros_now = recorded.to_sec()
            if baseline_time is None:
                baseline_time = ros_now
            now = ros_now - baseline_time + 10.
            if topic == '/v0_nav/status':
                task_state = json.loads(message.data).get('state')
                continue
            stamp = message.header.stamp.to_sec()
            if topic == '/mavros/state':
                old.state, old.state_time = (message.connected, message.armed, message.mode), now
                current.update_state(message.connected, message.armed, message.mode, stamp, now, ros_now)
                continue
            frames[message.header.frame_id] += 1
            if previous_rx is not None:
                gap = ros_now - previous_rx
                largest_gap = max(largest_gap, gap)
                if task_state in ('HOLD_INITIAL', 'EXEC_WAYPOINT', 'DWELL', 'FINISH_HOLD'):
                    largest_active_gap = max(largest_active_gap, gap)
                for threshold in (.35, .5):
                    if gap > threshold:
                        gap_counts[str(threshold)] += 1
            if previous_stamp is not None and stamp < previous_stamp:
                rewinds.append(dict(received_ros_s=ros_now, rewind_s=previous_stamp-stamp,
                                    previous_stamp_s=previous_stamp, stamp_s=stamp))
            previous_rx, previous_stamp = ros_now, stamp
            source_offsets.append(ros_now-stamp)
            position, quaternion = message.pose.position, message.pose.orientation
            arguments = ((position.x, position.y, position.z),
                         (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                         stamp, now, ros_now, message.header.frame_id)
            old.update_pose(*arguments)
            current.update_pose(*arguments)
            header_diagnostics[current.stamp_diagnostics.get('pose', 'NOT_ACCEPTED_WHILE_DISCONNECTED')] += 1
            details = dict(received_ros_s=ros_now, header_stamp_s=stamp, header_age_s=ros_now-stamp,
                           frame=message.header.frame_id, task_state=task_state)
            if old.fault and first_legacy_fault is None:
                first_legacy_fault = dict(reason=old.fault, **details)
            if current.fault and first_current_fault is None:
                first_current_fault = dict(reason=current.fault, **details)
    return dict(run=directory.name, historical_result=historical['result'],
                historical_error=historical.get('error'), bag_sha256=sha256(directory / 'flight.bag'),
                mission_yaml_sha256=sha256(directory / 'mission.yaml'),
                original_pose_timeout_s=baseline_config['safety']['pose_timeout_s'],
                comparison_pose_timeout_s=current_config['safety']['pose_timeout_s'],
                frames=dict(frames), first_legacy_pose_fault=first_legacy_fault,
                first_current_pose_fault=first_current_fault, stamp_diagnostics=dict(header_diagnostics),
                raw_stamp_rewinds=rewinds, header_age_range_s=[min(source_offsets), max(source_offsets)],
                max_bag_receipt_gap_s=largest_gap, max_active_bag_receipt_gap_s=largest_active_gap,
                bag_receipt_gaps_exceeding_s=dict(gap_counts),
                interpretation='Pose callback comparison only, not mission PASS. Bag receipt time is not the original node callback time; data delivery/EKF quality and physical correctness are not inferred from header diagnostics.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', default='af681a7cedf1e5dc532355aa5d261b11f1948437')
    parser.add_argument('--runs', nargs='+', default=[
        '20261027T143500-single', '20261027T144135-mavros_loss', '20261026T193632-multi'])
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    source = subprocess.check_output(['git', '-C', str(ROOT), 'show',
                                      args.baseline + ':src/indoor_nav/src/indoor_nav/v0.py'], text=True)
    baseline = types.ModuleType('v0_header_gate_baseline')
    exec(compile(source, 'git:' + args.baseline + ':v0.py', 'exec'), baseline.__dict__)
    result = dict(baseline_commit=args.baseline, baseline_source_sha256=hashlib.sha256(source.encode()).hexdigest(),
                  current_source_sha256=sha256(ROOT / 'src/indoor_nav/src/indoor_nav/v0.py'),
                  method='offline read-only bag traversal; legacy YAML deadlines unchanged; no ROS master, player, publishers, service calls or aircraft control',
                  runs=[replay(ROOT / 'docs/v0/evidence' / name, baseline) for name in args.runs])
    text = json.dumps(result, indent=2, ensure_ascii=False)
    if args.output:
        with args.output.open('x') as output:
            output.write(text + '\n')
    else:
        print(text)


if __name__ == '__main__':
    main()
