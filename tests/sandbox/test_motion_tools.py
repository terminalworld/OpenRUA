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
