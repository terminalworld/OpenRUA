"""Starter commands preserve documented arguments and native action failures."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

from openrua.sandbox import workspace


TOOLS = Path(workspace.__file__).parent / 'workspace/tools/action'


def tool(name):
    spec = importlib.util.spec_from_file_location(name, TOOLS / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('options,point,seconds', [
    ([], 'hand', 4.0), (['--at', 'tcp'], 'tcp', 4.0),
    (['2', '--at', 'tcp'], 'tcp', 2.0), (['--at=hand', '3'], 'hand', 3.0),
])
def test_ik_target_options_do_not_become_coordinates(options, point, seconds):
    parsed = tool('ik_move').parse_args(['0.1', '-0.2', '0.3', '0', '1', '0', '0', *options])
    assert (parsed.x, parsed.y, parsed.z) == (.1, -.2, .3)
    assert (parsed.qx, parsed.qy, parsed.qz, parsed.qw) == (0, 1, 0, 0)
    assert parsed.at == point and parsed.seconds == seconds


@pytest.mark.parametrize('options', [['--at', 'unknown'], ['--at'], ['--unknown']])
def test_ik_rejects_invalid_options_before_ros(options):
    with pytest.raises(SystemExit) as error:
        tool('ik_move').main(['0', '0', '0', '0', '0', '0', '1', *options])
    assert error.value.code == 2


@pytest.mark.parametrize('accepted,status,code,expected', [
    (True, 4, 0, 0), (True, 4, -5, 1),
    (True, 6, 0, 1), (True, 5, 0, 1), (False, None, None, 1),
])
def test_trajectory_exit_reflects_native_result(monkeypatch, capsys, accepted, status, code, expected):
    module = tool('fjt_send')
    calls = []
    outcome = NS(status=status, result=NS(error_code=code, error_string='native diagnostic'))
    class Handle:
        def __init__(self):
            self.accepted = accepted
        def get_result_async(self):
            assert accepted, 'a rejected goal has no result to await'
            return NS(result=lambda: outcome)
    class Client:
        def __init__(self, *args):
            pass
        def wait_for_server(self, **kwargs):
            return True
        def send_goal_async(self, goal):
            calls.append(goal)
            return NS(result=Handle)
    node = NS(destroy_node=lambda: calls.append('destroy'))
    modules = {
        'rclpy': NS(init=lambda: None, create_node=lambda _: node,
                    spin_until_future_complete=lambda *a, **kw: None,
                    shutdown=lambda: calls.append('shutdown')),
        'rclpy.action': NS(ActionClient=Client),
        'action_msgs.msg': NS(GoalStatus=NS(STATUS_SUCCEEDED=4)),
        'builtin_interfaces.msg': NS(Duration=NS),
        'control_msgs.action': NS(FollowJointTrajectory=NS(Goal=lambda: NS(trajectory=NS()))),
        'trajectory_msgs.msg': NS(JointTrajectoryPoint=NS),
    }
    for name, value in modules.items():
        monkeypatch.setitem(sys.modules, name, value)
    monkeypatch.setattr(sys, 'argv', ['fjt_send.py', '0.1,-0.2', '2'])
    monkeypatch.setattr(module, 'manifest_entry', lambda: {'joints': ['a', 'b'], 'port': '/trajectory'})
    assert module.main() == expected
    assert calls[0].trajectory.joint_names == ['a', 'b']
    assert calls[0].trajectory.points[0].positions == [.1, -.2]
    assert calls[-2:] == ['destroy', 'shutdown']
    output = capsys.readouterr()
    if not accepted:
        assert 'rejected' in output.err
    elif expected:
        assert 'native diagnostic' in output.err
    else:
        assert not output.err


@pytest.mark.parametrize('point,expected_z', [('hand', .3), ('tcp', .2)])
def test_ik_requests_the_manifest_hand_link(monkeypatch, point, expected_z):
    module = tool('ik_move')
    requests = []
    manifest = {'frames': {'hand': 'custom_hand'}, 'hand': {'tcp_offset_m': .1},
                'planning': {'group': 'custom_arm', 'ik_service': '/ik'},
                'actuators': [{'kind': 'joint_trajectory', 'joints': ['joint_a']}]}
    joint_state = NS(name=['joint_a'], position=[.5])
    result = NS(error_code=NS(val=1), solution=NS(joint_state=joint_state))
    class Client:
        def wait_for_service(self, **kwargs):
            return True
        def call_async(self, request):
            requests.append(request)
            return NS(result=lambda: result)
    class Node:
        def create_subscription(self, kind, topic, callback, qos):
            callback(joint_state)
        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=0))
        def create_client(self, kind, name):
            assert name == '/ik'
            return Client()
    def request():
        pose = NS(position=NS(), orientation=NS())
        return NS(ik_request=NS(ik_link_name='', robot_state=NS(),
                               pose_stamped=NS(header=NS(), pose=pose)))
    monkeypatch.setitem(sys.modules, 'rclpy', NS(
        init=lambda: None, create_node=lambda _: Node(),
        spin_until_future_complete=lambda *a, **kw: None, shutdown=lambda: None))
    monkeypatch.setitem(sys.modules, 'moveit_msgs.srv', NS(GetPositionIK=NS(Request=request)))
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', NS(JointState=lambda: NS(name=[], position=[])))
    monkeypatch.setattr(module, 'manifest', lambda: manifest)
    import subprocess
    monkeypatch.setattr(subprocess, 'run', lambda *a, **kw: NS(returncode=0))
    with pytest.raises(SystemExit) as error:
        module.main(['.1', '.2', '.3', '0', '0', '0', '1', '--at', point])
    assert error.value.code == 0
    assert requests[0].ik_request.ik_link_name == 'custom_hand'
    assert requests[0].ik_request.group_name == 'custom_arm'
    assert requests[0].ik_request.pose_stamped.pose.position.z == pytest.approx(expected_z)
