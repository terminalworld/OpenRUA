"""Exercise one simulation startup, ROS observations and a no-op control request.

Run after building the declared simulator and sandbox images. No model calls,
credentials, physical hardware or existing sessions are used. Only a successful
run writes evidence eligible for the guided startup menu.
"""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import tempfile
import uuid

from openrua import agents, config
from openrua.config.startup import profile_digest
from openrua.runner.bringup import bring_up, ensure_internal_network, start_episode
from openrua.runner.live import stop_resources

PROBE = r"""
import json, time
import rclpy
from sensor_msgs.msg import JointState, Image
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
node = rclpy.create_node('startup_probe')
seen = {}
node.create_subscription(JointState, '/joint_states', lambda msg: seen.update(joints=msg), qos_profile_sensor_data)
def wait(predicate, label, seconds=120):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
        if predicate(): return
    raise RuntimeError('timeout waiting for ' + label)
wait(lambda: 'joints' in seen, 'joint states')
wait(lambda: any('sensor_msgs/msg/Image' in types for _, types in node.get_topic_names_and_types()), 'camera topic')
topic = next(name for name, types in node.get_topic_names_and_types() if 'sensor_msgs/msg/Image' in types and '/color/' in name)
node.create_subscription(Image, topic, lambda msg: seen.update(image=msg), qos_profile_sensor_data)
wait(lambda: 'image' in seen, 'camera frame')
assert seen['image'].width > 0 and len(seen['image'].data) > 0
# The command parameters are taken from the same resolved configuration as bring-up.
cfg = json.loads(__import__('sys').argv[1])
arm = cfg['arms'][0] if cfg.get('arms') else cfg['arm']
ports = arm.get('ports', cfg.get('ports', {}))
client = ActionClient(node, FollowJointTrajectory, ports['trajectory'])
assert client.wait_for_server(timeout_sec=60), 'trajectory action unavailable'
state = dict(zip(seen['joints'].name, seen['joints'].position))
goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = arm['joints']
point = JointTrajectoryPoint()
point.positions = [state[name] for name in arm['joints']]
point.time_from_start = Duration(sec=1)
goal.trajectory.points = [point]
future = client.send_goal_async(goal)
wait(future.done, 'trajectory acceptance')
assert future.result().accepted, 'trajectory rejected'
result = future.result().get_result_async()
wait(result.done, 'trajectory result')
assert result.result().result.error_code == 0, str(result.result())
print(json.dumps({'joint_states': True, 'camera_frame': True, 'trajectory': True}))
node.destroy_node()
rclpy.shutdown()
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', required=True)
    parser.add_argument('--sim', required=True)
    parser.add_argument('--bench', default='')
    parser.add_argument('--agent', action='append', default=None,
                        help='agent CLI to check through its manifest, repeatable; default: the scene configuration')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    selection = dict(robot=args.robot, sim=args.sim, bench=args.bench)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # A failed rerun must not leave a previous pass advertised as current.
    args.output.write_text(json.dumps({'status': 'not_verified', 'selection': selection}) + '\n')
    with tempfile.TemporaryDirectory(prefix='openrua-startup-') as directory:
        root = Path(directory)
        composed = config.compose(args.robot, args.sim, args.bench, root)
        cfg = copy.deepcopy(composed.cfg)
        if cfg['machine']['backend']['kind'] != 'sim':
            parser.error('this check only operates simulated robots')
        config.apply_suite_overrides(cfg, composed.suite)
        config.normalize_arms(cfg)
        backend = cfg['machine']['backend']
        images = {}
        for role, image in [('robot', backend['image']), ('sandbox', backend['sandbox_image'])]:
            images[role] = {'name': image, 'id': subprocess.check_output(
                ['docker', 'image', 'inspect', '--format', '{{.Id}}', image], text=True).strip()}
        name = 'openrua-check-' + uuid.uuid4().hex[:12]
        network = ensure_internal_network(name)
        machine = None
        try:
            print('Starting ' + json.dumps(selection), flush=True)
            _, machine, _ = bring_up(cfg, root, name + '-robot', name + '-sandbox',
                composed.suite, composed.task_id, network, 'http://unused.invalid', (), None,
                root / 'robot.log', home=root)
            info = start_episode(machine, 0)
            assert info.get('language') or info.get('name'), 'scene returned neither a task instruction nor a scene name'
            probe = root / 'probe.py'
            probe.write_text(PROBE)
            subprocess.run(['docker', 'cp', str(probe), name + '-sandbox:/tmp/probe.py'], check=True)
            checked = subprocess.run(['docker', 'exec', name + '-sandbox', 'python3', '/tmp/probe.py',
                json.dumps(cfg['machine'])], capture_output=True, text=True, timeout=420)
            if checked.returncode:
                raise RuntimeError(checked.stdout + checked.stderr)
            checks = json.loads(checked.stdout.strip().splitlines()[-1])
            versions = {}
            selected = cfg['agent']
            for spec in args.agent or [selected['name']]:
                manifest = agents.manifest(spec, version=selected.get('version')
                                           if spec == selected['name'] else None)
                version_argv = manifest.fields.get('version_argv')
                if not version_argv:
                    continue
                result = subprocess.run(['docker', 'exec', name + '-sandbox', *version_argv],
                    capture_output=True, text=True, timeout=30, check=True)
                versions[manifest.name] = result.stdout.strip()
        except BaseException:
            log = root / 'robot.log'
            if log.exists(): print(log.read_text()[-10000:], flush=True)
            raise
        finally:
            try:
                if machine: stop_resources(name + '-sandbox', machine.shutdown)
            finally:
                subprocess.run(['docker', 'network', 'rm', network], check=False, capture_output=True)
        report = {'status': 'passed', 'selection': selection, 'suite': composed.suite,
                  'task_id': composed.task_id, 'seed': 0, 'profiles_sha256': profile_digest(),
                  'checked_at': datetime.now(timezone.utc).isoformat(), 'images': images,
                  'checks': checks, 'agent_binaries': versions,
                  'scope': 'Simulation startup, reset, ROS image and joint streams, no-op trajectory, agent CLI versions. No model task or physical robot validation.'}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print('PASS: ' + str(args.output), flush=True)


if __name__ == '__main__':
    main()
