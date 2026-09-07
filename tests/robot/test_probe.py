"""The profile draft: read from the graph where it can, TODO where it cannot."""

import yaml

from robocli.robot.real.probe import draft_profile, read_graph

URDF_FACTS = {"name": "ur5e", "root": "base_link",
              "joints": [{"name": "shoulder_pan_joint", "type": "revolute", "lower": -6.28, "upper": 6.28},
                         {"name": "elbow_joint", "type": "revolute", "lower": -3.14, "upper": 3.14},
                         {"name": "finger_joint", "type": "prismatic", "lower": 0.0, "upper": 0.085}],
              "links": ["base_link", "forearm_link", "tool0"]}


def _fake_run(cmd: str) -> str:
    if cmd == "ros2 topic list":
        return "/joint_states\n/servo_node/delta_twist_cmds\n/wrist_camera/color/image_raw\n/force_torque_sensor_broadcaster/wrench\n"
    if cmd == "ros2 action list":
        return "/scaled_joint_trajectory_controller/follow_joint_trajectory\n/robotiq_gripper_controller/gripper_cmd\n/move_action\n"
    if cmd == "ros2 service list":
        return "/compute_ik\n"
    import json
    return json.dumps(URDF_FACTS)


def test_draft_reads_ports_joints_and_frames_from_the_graph():
    graph = read_graph(_fake_run)
    text = draft_profile(graph, {"network": "host"})
    d = yaml.safe_load(text)["machine"]
    assert d["backend"] == {"kind": "real", "ros_distro": "jazzy", "discovery": {"network": "host"}}
    assert d["arm"]["joints"] == ["shoulder_pan_joint", "elbow_joint"]
    assert d["arm"]["limits_rad"] == [[-6.28, 6.28], [-3.14, 3.14]]
    assert d["gripper"]["open_m"] == 0.085 and d["gripper"]["closed_m"] == 0.0
    assert d["frames"]["base"] == "base_link" and d["frames"]["hand"] == "tool0"
    assert d["ports"]["trajectory"].endswith("follow_joint_trajectory")
    assert d["ports"]["gripper"].endswith("gripper_cmd")
    assert d["ports"]["twist"] == "/servo_node/delta_twist_cmds"
    assert d["planning"]["moveit"] is True and d["cameras"]["list"] == ["wrist_camera"]
    assert "TODO" in text


def test_draft_without_a_urdf_marks_the_arm_todo():
    def run(cmd):
        return "/joint_states\n" if cmd == "ros2 topic list" else ""
    graph = read_graph(run)
    assert graph["urdf"] is None
    text = draft_profile(graph, {"static_peers": ["10.0.0.2"]})
    d = yaml.safe_load(text)["machine"]
    assert d["backend"]["discovery"] == {"static_peers": ["10.0.0.2"]}
    assert d["arm"]["joints"] == [] and "# TODO: no revolute joints" in text
    assert "trajectory" not in d["ports"] and "planning" not in d
