#!/usr/bin/env python3
"""Offline evidence statistics; never replays control messages to ROS."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path

import rosbag


def distribution(values):
    if not values:
        return None
    ordered = sorted(values)
    return {'count': len(values), 'max': ordered[-1],
            'p99': ordered[min(len(ordered)-1, int(.99 * len(ordered)))],
            'rms': math.sqrt(sum(value * value for value in values) / len(values))}


def speed_statistics(rows, threshold):
    samples = [(row['mono_s'], math.hypot(*row['velocity'][:2])) for row in rows if 'velocity' in row]
    cumulative = longest = current = 0.
    unobserved = 0.
    for (before, speed_before), (after, speed_after) in zip(samples, samples[1:]):
        duration = after-before
        if duration > .25:
            unobserved += duration
            current = 0.
            continue
        if speed_before > threshold and speed_after > threshold:
            cumulative += duration
            current += duration
            longest = max(longest, current)
        else:
            current = 0.
    return {'threshold_mps': threshold, 'peak_mps': max((speed for _, speed in samples), default=0.),
            'observed_above_threshold_s': cumulative, 'longest_observed_continuous_s': longest,
            'unobserved_gap_s': unobserved, 'method': 'sample-based; adjacent samples > threshold, gaps >0.25s excluded; not a continuous-time bound'}


def analyze(directory):
    counts = defaultdict(int)
    stamps = defaultdict(list)
    sequences = defaultdict(list)
    payload_sizes = defaultdict(set)
    velocity_rows = []
    current_status = {}
    with rosbag.Bag(str(directory / 'flight.bag')) as bag:
        for topic, message, stamp in bag.read_messages():
            counts[topic] += 1
            stamps[topic].append(stamp.to_sec())
            if topic == '/v0_nav/status':
                try:
                    current_status = json.loads(message.data)
                except ValueError:
                    current_status = {}
            if topic == '/mavros/local_position/velocity_local':
                velocity_rows.append({'mono_s': stamp.to_sec(),
                                      'velocity': [message.twist.linear.x, message.twist.linear.y, message.twist.linear.z],
                                      'status': dict(current_status)})
            if topic in ('/sound_sensor/audio_raw', '/motor_mic/audio_raw', '/vibration_data') and message._type == 'std_msgs/UInt8MultiArray':
                payload = bytes(message.data)
                sequences[topic].append(int.from_bytes(payload[:8], 'little'))
                payload_sizes[topic].add(len(payload))
    summary = {'bag_readable': True, 'topics': {}}
    for topic, count in counts.items():
        times = stamps[topic]
        data = {'messages': count, 'max_gap_s': max((later-earlier for earlier, later in zip(times, times[1:])), default=0),
                'rate_hz': (count-1)/(times[-1]-times[0]) if count > 1 and times[-1] > times[0] else 0}
        if topic in sequences:
            sequence = sequences[topic]
            data['payload_bytes'] = sorted(payload_sizes[topic])
            data['sequence_missing_inside_epochs'] = sum(max(0, later-earlier-1) for earlier, later in zip(sequence, sequence[1:]) if later > earlier)
            data['sequence_resets'] = sum(later <= earlier for earlier, later in zip(sequence, sequence[1:]))
        summary['topics'][topic] = data
    rows = [json.loads(line) for line in (directory / 'telemetry.jsonl').read_text().splitlines()]
    phases = defaultdict(list)
    active = []
    for row in rows:
        if row.get('status', {}).get('state') in ('EXEC_WAYPOINT', 'DWELL', 'FINISH_HOLD'):
            active.append(row)
            phases[row['status']['wp']].append(row)
    summary['pose_header_age_s'] = distribution([row['pose_age_s'] for row in active if 'pose_age_s' in row])
    summary['speed'] = {str(threshold): speed_statistics(active, threshold) for threshold in (.2, .3)}
    bag_active = [row for row in velocity_rows if row['status'].get('state') in ('HOLD_INITIAL', 'EXEC_WAYPOINT', 'DWELL', 'FINISH_HOLD')]
    summary['bag_velocity_speed'] = {str(threshold): speed_statistics(bag_active, threshold) for threshold in (.2, .3)}
    summary['bag_velocity_timing'] = 'ROS bag receipt time, measured velocity_local; telemetry speed uses monotonic samples; no continuous-time guarantee'
    summary['bag_velocity_by_wp'] = {str(waypoint): speed_statistics([row for row in bag_active if row['status'].get('wp') == waypoint], .2)
                                     for waypoint in sorted({row['status'].get('wp') for row in bag_active if isinstance(row['status'].get('wp'), int)})}
    summary['brake_speed'] = speed_statistics([row for row in rows if row.get('mode') == 'BRAKE'], .2)
    summary['speed_by_state'] = {state: speed_statistics([row for row in rows if row.get('status', {}).get('state') == state], .2)
                                for state in ('HOLD_INITIAL', 'EXEC_WAYPOINT', 'DWELL', 'FINISH_HOLD')}
    summary['gate_publish_gap_s'] = max((row.get('watchdog', {}).get('max_publish_gap_s', 0.) for row in rows), default=0.)
    summary['waypoint_tracking'] = {}
    for waypoint, samples in phases.items():
        errors = [math.hypot(*(row['pose'][index]-row['target'][index] for index in range(2)))
                  for row in samples if 'pose' in row and 'target' in row]
        truth = [math.hypot(*(row['truth'][index]-row['expected_truth'][index] for index in range(2)))
                 for row in samples if 'truth' in row and 'expected_truth' in row]
        summary['waypoint_tracking'][waypoint] = {'sample_count': len(samples),
                                                'fcu_target_xy_error_m': distribution(errors),
                                                'truth_target_xy_error_m': distribution(truth)}
    events = [json.loads(line) for line in (directory / 'events.jsonl').read_text().splitlines()]
    summary['state_events'] = [{key: event[key] for key in ('state', 'wp', 'ros_s', 'mono_delta_s', 'reason') if key in event}
                               for event in events if event.get('event') == 'TRANSITION']
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    result = analyze(args.directory)
    (args.directory / 'bag_analysis.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({'directory': str(args.directory), 'bag_readable': result['bag_readable'],
                      'message_count': sum(topic['messages'] for topic in result['topics'].values())}))
