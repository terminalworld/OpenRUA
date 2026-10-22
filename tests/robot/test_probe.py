"""The profile draft: read from the graph where it can, TODO where it cannot."""

import pytest
import yaml

from openrua.robot.real.probe import draft_profile, read_graph

# A Panda in DROID configuration as the lab's driver serves it: the arm's
# trajectory controller, a Robotiq gripper (a revolute knuckle joint) in
# its own namespace, MoveIt, a RealSense D405 and a wrist ZED Mini.
URDF_FACTS = {"name": "panda_droid", "root": "panda_link0",
              "joints": [{"name": "panda_joint1", "type": "revolute", "lower": -2.9, "upper": 2.9},
                         {"name": "panda_joint2", "type": "revolute", "lower": -1.76, "upper": 1.76},
                         {"name": "robotiq_85_left_knuckle_joint", "type": "revolute",
                          "lower": 0.0, "upper": 0.8}],
              "links": ["panda_link0", "panda_link8", "robotiq_85_base_link"]}
SRDF_FACTS = {"groups": {"panda_arm": {"joints": [], "chains": [["panda_link0", "panda_link8"]]}},
              "end_effectors": [], "virtual_joints": []}
TOPICS = """/joint_states [sensor_msgs/msg/JointState]
/franka_robot_state_broadcaster/external_wrench_in_base_frame [geometry_msgs/msg/WrenchStamped]
/d405/color/image_raw [sensor_msgs/msg/Image]
/d405/color/camera_info [sensor_msgs/msg/CameraInfo]
/d405/aligned_depth_to_color/image_raw [sensor_msgs/msg/Image]
/zed_wrist/zed_node/rgb/image_rect_color [sensor_msgs/msg/Image]
/zed_wrist/zed_node/rgb/camera_info [sensor_msgs/msg/CameraInfo]
/zed_wrist/zed_node/depth/depth_registered [sensor_msgs/msg/Image]
"""
ACTIONS = """/joint_trajectory_controller/follow_joint_trajectory [control_msgs/action/FollowJointTrajectory]
/gripper/robotiq_gripper_controller/gripper_cmd [control_msgs/action/GripperCommand]
/move_action [moveit_msgs/action/MoveGroup]
"""


def _fake_run(cmd: str) -> str:
    import json
    if cmd == "ros2 topic list -t":
        return TOPICS
    if cmd == "ros2 action list -t":
        return ACTIONS
    if cmd == "ros2 service list":
        return "/compute_ik\n/compute_fk\n"
    if cmd.startswith("ros2 param get /joint_trajectory_controller joints"):
        return "String values are: ['panda_joint1', 'panda_joint2']\n"
    if cmd.startswith("ros2 param get /gripper/robotiq_gripper_controller joint"):
        return "String value is: robotiq_85_left_knuckle_joint\n"
    if "robot_description_semantic" in cmd:
        return json.dumps(SRDF_FACTS)
    return json.dumps(URDF_FACTS)


def test_draft_reads_ports_joints_and_frames_from_the_graph():
    graph = read_graph(_fake_run)
    text = draft_profile(graph, {"network": "host"})
    d = yaml.safe_load(text)["machine"]
    assert d["backend"] == {"kind": "real", "ros_distro": "jazzy", "discovery": {"network": "host"}}
    # the arm is what its trajectory controller drives; the knuckle is not
    assert d["arm"]["joints"] == ["panda_joint1", "panda_joint2"]
    assert d["arm"]["limits_rad"] == [[-2.9, 2.9], [-1.76, 1.76]]
    assert d["gripper"]["joint"] == "robotiq_85_left_knuckle_joint"
    assert "revolute and travels 0.0 .. 0.8" in text
    assert d["frames"]["base"] == "panda_link0" and d["frames"]["hand"] == "panda_link8"
    assert d["planning"]["group"] == "panda_arm" and d["planning"]["moveit"] is True
    assert d["ports"]["trajectory"].endswith("follow_joint_trajectory")
    assert d["ports"]["gripper"] == "/gripper/robotiq_gripper_controller/gripper_cmd"
    assert d["ports"]["wrench"].endswith("external_wrench_in_base_frame")
    assert [c["name"] for c in d["cameras"]["list"]] == ["d405", "zed_wrist"]
    assert "/d405/aligned_depth_to_color/image_raw" in text


def test_what_only_the_owner_knows_fails_validation_until_filled_in():
    from openrua import config
    graph = read_graph(_fake_run)
    d = yaml.safe_load(draft_profile(graph, {"network": "host"}))
    m = d["machine"]
    assert m["gripper"]["open"] == m["gripper"]["holds_partway"] == "TODO"
    assert m["frames"]["world"] == "TODO" and m["cameras"]["list"][0]["type"] == "TODO"
    with pytest.raises(config.ConfigError):
        config.validate(config.RobotInstance, d, "draft")


def test_draft_without_a_urdf_marks_the_arm_todo():
    def run(cmd):
        return "/joint_states [sensor_msgs/msg/JointState]\n" if cmd == "ros2 topic list -t" else ""
    graph = read_graph(run)
    assert graph["urdf"] is None
    text = draft_profile(graph, {"static_peers": ["10.0.0.2"]})
    d = yaml.safe_load(text)["machine"]
    assert d["backend"]["discovery"] == {"static_peers": ["10.0.0.2"]}
    assert d["arm"]["joints"] == [] and "# TODO: no trajectory controller" in text
    assert "trajectory" not in d["ports"] and "planning" not in d
    assert d["cameras"] == {"list": []}


def test_explicit_ros_release_is_preserved_in_profile():
    graph = read_graph(_fake_run)
    text = draft_profile(graph, {"network": "host"}, ros_distro="humble")
    assert yaml.safe_load(text)["machine"]["backend"]["ros_distro"] == "humble"


@pytest.mark.parametrize("separator", ["", "\n---\n"])
def test_native_urdf_converter_accepts_ros_topic_document_separator(monkeypatch, capsys, separator):
    import io
    import json
    import sys
    import types
    import xml.etree.ElementTree as ET
    from openrua.robot.real.probe import _URDF_FACTS

    class ParsedRobot:
        @staticmethod
        def from_xml_string(text):
            root = ET.fromstring(text)
            return types.SimpleNamespace(name=root.attrib["name"], joints=[], links=[],
                                         get_root=lambda: "base")

    package = types.ModuleType("urdf_parser_py")
    module = types.ModuleType("urdf_parser_py.urdf")
    module.URDF = ParsedRobot
    monkeypatch.setitem(sys.modules, "urdf_parser_py", package)
    monkeypatch.setitem(sys.modules, "urdf_parser_py.urdf", module)
    monkeypatch.setattr(sys, "stdin", io.StringIO('<robot name="fixture"><link name="base"/></robot>' + separator))
    exec(_URDF_FACTS, {})
    assert json.loads(capsys.readouterr().out)["name"] == "fixture"


def test_a_machine_with_two_arms_is_drafted_arm_by_arm():
    import json
    urdf = {"name": "duo", "root": "world", "links": [],
            "joints": [{"name": f"{side}_joint{i}", "type": "revolute", "lower": -2.9, "upper": 2.9,
                        "parent": f"{side}_base_link" if i == 1 else f"{side}_link{i - 1}",
                        "child": f"{side}_link{i}"}
                       for side in ("left", "right") for i in (1, 2)]
                      + [{"name": f"{side}_finger_joint", "type": "prismatic", "lower": 0.0,
                          "upper": 0.04, "parent": f"{side}_hand", "child": f"{side}_finger"}
                         for side in ("left", "right")]}

    def run(cmd):
        if cmd == "ros2 topic list -t":
            return "/joint_states [sensor_msgs/msg/JointState]\n"
        if cmd == "ros2 action list -t":
            return "".join(f"/{side}_arm_controller/follow_joint_trajectory [control_msgs/action/FollowJointTrajectory]\n"
                           f"/{side}_gripper_controller/gripper_cmd [control_msgs/action/GripperCommand]\n"
                           for side in ("right", "left"))
        if cmd == "ros2 service list":
            return ""
        for side in ("left", "right"):
            if cmd.startswith(f"ros2 param get /{side}_arm_controller joints"):
                return f"String values are: ['{side}_joint1', '{side}_joint2']\n"
            if cmd.startswith(f"ros2 param get /{side}_gripper_controller joint"):
                return f"String value is: {side}_finger_joint\n"
        if "robot_description_semantic" in cmd:
            return ""
        return json.dumps(urdf)

    text = draft_profile(read_graph(run), {"network": "host"})
    m = yaml.safe_load(text)["machine"]
    assert "arm" not in m and "ports" not in m
    left, right = m["arms"]
    assert (left["label"], right["label"]) == ("left", "right")
    assert right["joints"] == ["right_joint1", "right_joint2"]
    assert right["base_frame"] == "right_base_link"
    assert right["gripper"]["joint"] == "right_finger_joint"
    assert right["ports"] == {"trajectory": "/right_arm_controller/follow_joint_trajectory",
                              "gripper": "/right_gripper_controller/gripper_cmd"}
    assert left["hand_frame"] == "TODO" and right["gripper"]["unit"] == "TODO"
