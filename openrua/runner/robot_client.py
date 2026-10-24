#!/usr/bin/env python3
"""Inspect or maintain this OpenRUA session without leaving its conversation.

Only the selected robot's declared resources are managed. This is not a
physical emergency stop. Restart creates a new driver from current configuration;
old tasks are not replayed. If the ROS tool environment changes, finish the
current agent turn after acceptance so OpenRUA can perform an internal handover.
"""

import argparse
import json
from pathlib import Path
import socket
import shutil
import time
import uuid


INSTRUCTIONS = """
## Connection maintenance

This conversation remains available when the robot is disconnected or its
startup fails. Inspect current connection status before issuing robot commands:

    python3 /openrua-runtime/robot.py status
    python3 /openrua-runtime/robot.py logs
    python3 /openrua-runtime/robot.py build --wait
    python3 /openrua-runtime/robot.py restart --wait

These operations apply only to this session's configured resources. Build uses
the robot's declared image recipes. Restart reads maintained configuration and
creates a new driver; it does not restart OpenRUA or replay prior tasks. Retain
the printed request ID and inspect status before retrying a lost response.
If status says waiting for a tool-environment handover, finish this turn; the
same conversation continues internally. A driver stop follows its declared
shutdown contract and is not a physical emergency stop. After reconnection,
read current machine information and observations before further robot work.
"""


def prepare_client(directory):
    directory.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(Path(__file__), directory / 'robot.py')
    (directory / 'README.md').write_text(INSTRUCTIONS)


def guide(materials):
    for name in ('README.md', 'AGENTS.md'):
        path = materials / name
        if path.is_file() and '## Connection maintenance' not in path.read_text():
            with path.open('a') as stream:
                stream.write('\n' + INSTRUCTIONS)


def call(path, body):
    with socket.socket(socket.AF_UNIX) as client:
        client.settimeout(15)
        client.connect(str(path))
        client.sendall((json.dumps(body) + '\n').encode())
        with client.makefile('rb') as stream:
            result = json.loads(stream.readline(1024 * 1024))
    if 'error' in result:
        raise RuntimeError(result['error'])
    return result['result']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('status', 'logs', 'start', 'stop', 'restart', 'build'))
    parser.add_argument('--request-id', help='reuse with identical operation after a lost acknowledgement')
    parser.add_argument('--wait', action='store_true', help='wait for driver/build results; returns on environment handover')
    args = parser.parse_args()
    identity = args.request_id or uuid.uuid4().hex
    body = {'operation': args.operation}
    if args.operation not in {'status', 'logs'} or args.request_id:
        body['request_id'] = identity
    if args.operation not in {'status', 'logs'}:
        print('request_id=' + identity, flush=True)
    try:
        path = Path(__file__).with_name('control.sock')
        result = call(path, body)
        while args.wait and result.get('status') in {'accepted', 'running'}:
            time.sleep(1)
            result = call(path, {'operation': 'status', 'request_id': identity})
        print(json.dumps(result, indent=2))
        return 1 if result.get('status') in {'failed', 'unknown'} else 0
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f'{exc}. Inspect status with the same request ID before retrying.\n')


if __name__ == '__main__':
    raise SystemExit(main())
