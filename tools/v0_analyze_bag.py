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


def analyze(directory):
    counts = defaultdict(int)
    stamps = defaultdict(list)
    sequences = defaultdict(list)
    payload_sizes = defaultdict(set)
    with rosbag.Bag(str(directory / 'flight.bag')) as bag:
        for topic, message, stamp in bag.read_messages():
            counts[topic] += 1
            stamps[topic].append(stamp.to_sec())
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
