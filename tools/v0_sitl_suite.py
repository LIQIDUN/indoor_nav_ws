#!/usr/bin/env python3
"""Identity-gated cold-instance runner; stops only DISARMED owned simulations."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
MASTER = 'http://127.0.0.1:11421'


def ports_free():
    for kind, ports in ((socket.SOCK_STREAM, (11421, 11455, 5760)), (socket.SOCK_DGRAM, (9002, 9003, 14672))):
        for port in ports:
            with socket.socket(socket.AF_INET, kind) as probe:
                try:
                    probe.bind(('0.0.0.0', port))
                except OSError:
                    return False
    return True


def wait_for(condition, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(.5)
    raise RuntimeError('readiness deadline exceeded')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sitl', action='store_true', required=True)
    parser.add_argument('--execute', action='store_true', required=True)
    parser.add_argument('--cases', nargs='+', required=True)
    parser.add_argument('--mission-file', type=Path, default=ROOT / 'src/indoor_nav/config/mission_v0.yaml')
    args = parser.parse_args()
    if os.environ.get('ROS_MASTER_URI') != MASTER:
        raise RuntimeError('wrong master; source the ROS workspace and set the isolated master')
    socket.setdefaulttimeout(2.)
    directory = ROOT / 'docs/v0/evidence' / ('resume-suite-' + time.strftime('%Y%m%dT%H%M%S') + '-' + str(os.getpid()))
    directory.mkdir(exist_ok=False)
    results = []
    master = xmlrpc.client.ServerProxy(MASTER)
    for index, case in enumerate(args.cases):
        wait_for(ports_free, 75)
        known = set((ROOT / '.sitl_runtime/v0').iterdir())
        console = (directory / ('{:02d}-{}-instance.txt'.format(index, case))).open('x')
        launcher = subprocess.Popen([str(ROOT / 'tools/v0_sitl_instance.sh'), '--sitl'], cwd=ROOT,
                                    stdout=console, stderr=subprocess.STDOUT)
        runtime = None
        def ready():
            nonlocal runtime
            if launcher.poll() is not None:
                raise RuntimeError('owned launcher exited before ready')
            try:
                code, _, value = master.getParam('/v0_suite', '/v0_sitl/runtime')
                candidate = Path(value).resolve() if code == 1 else None
                code, _, route = master.getParam('/v0_suite', '/mavros/fcu_url')
                if candidate and candidate.parent == ROOT / '.sitl_runtime/v0' and candidate not in known and code == 1 and route == 'udp://127.0.0.1:14672@':
                    runtime = candidate
                    return True
            except (OSError, xmlrpc.client.Error):
                pass
            return False
        try:
            wait_for(ready, 65)
            time.sleep(6)
            before = set((ROOT / 'docs/v0/evidence').glob('*/summary.json'))
            with (directory / ('{:02d}-{}-test.txt'.format(index, case))).open('x') as output:
                test = subprocess.run(['python3', str(ROOT / 'tools/v0_sitl_test.py'), '--sitl', '--execute', '--case', case,
                                       '--mission-file', str(args.mission_file.expanduser().resolve())],
                                      cwd=ROOT, stdout=output, stderr=subprocess.STDOUT, timeout=240)
            created = set((ROOT / 'docs/v0/evidence').glob('*/summary.json')) - before
            row = {'case': case, 'test_exit_code': test.returncode, 'runtime': str(runtime),
                   'summaries': [str(path.relative_to(ROOT)) for path in sorted(created)], 'launcher_pid': launcher.pid}
            cleanup = json.loads(next(iter(created)).parent.joinpath('cleanup.json').read_text()) if created else {}
            if cleanup.get('sim_stopped_by_test'):
                row['stop_exit_code'], row['stop_method'] = 0, 'test stopped verified DISARMED simulation before closing replacement MAVROS'
            else:
                with (directory / ('{:02d}-{}-stop.txt'.format(index, case))).open('x') as output:
                    stop = subprocess.run(['python3', str(ROOT / 'tools/v0_sitl_stop.py'), '--sitl'], cwd=ROOT,
                                          stdout=output, stderr=subprocess.STDOUT, timeout=70)
                row['stop_exit_code'] = stop.returncode
            if row['stop_exit_code']:
                results.append(row)
                raise RuntimeError('safe stop refused; leaving instance untouched, no next flight')
            launcher.wait(timeout=25)
            wait_for(ports_free, 75)
            row['ports_released'] = True
            results.append(row)
            (directory / 'results.json').write_text(json.dumps(results, indent=2))
            print(json.dumps(row), flush=True)
        except Exception as error:
            (directory / 'incomplete.json').write_text(json.dumps({'error': str(error), 'runtime': str(runtime),
                            'launcher_pid': launcher.pid, 'completed': results}, indent=2))
            raise
        finally:
            console.close()
    raise SystemExit(0 if all(row['test_exit_code'] == 0 for row in results) else 1)


if __name__ == '__main__':
    main()
