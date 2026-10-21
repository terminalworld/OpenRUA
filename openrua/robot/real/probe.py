"""Read a live ROS 2 graph and draft a robot profile from it.

What a person would otherwise type into a profile by hand is mostly
already on the graph: the topics, actions and services the robot serves
(with their types), the joints each controller drives, the joint limits
and frames in ``/robot_description``, and the planning groups in
``/robot_description_semantic``. This module reads them through a
caller-supplied ``run`` (a shell executed where the graph is visible,
the sandbox) and prints a profile draft.

What only the robot's owner knows stays ``TODO``: which end of the
gripper's travel is open, what its driver does with a goal, the world
frame, each camera's type and mount. A TODO is not a valid value, so the
draft fails validation until a person decides.
"""

from __future__ import annotations

import ast
import json
from typing import Callable

Run = Callable[[str], str]

# The python that turns the published URDF into the facts a profile
# carries; runs where urdfdom-py is installed (the sandbox image).
_URDF_FACTS = r"""
import json, sys
from urdf_parser_py.urdf import URDF
# ros2 topic echo appends a YAML document separator after the XML field.
xml = sys.stdin.read().strip().removesuffix("\n---")
r = URDF.from_xml_string(xml)
joints = [j for j in r.joints if j.type in ("revolute", "prismatic", "continuous")]
print(json.dumps({
    "name": r.name,
    "root": r.get_root(),
    "joints": [{"name": j.name, "type": j.type,
                "lower": getattr(j.limit, "lower", None) if j.limit else None,
                "upper": getattr(j.limit, "upper", None) if j.limit else None}
               for j in joints],
    "links": [l.name for l in r.links],
}))
"""

# The python that reads the planning groups out of the published SRDF.
_SRDF_FACTS = r"""
import json, sys
import xml.etree.ElementTree as ET
xml = sys.stdin.read().strip().removesuffix("\n---")
root = ET.fromstring(xml)
groups = {}
for g in root.findall("group"):
    groups[g.get("name")] = {
        "joints": [j.get("name") for j in g.findall("joint")],
        "chains": [[c.get("base_link"), c.get("tip_link")] for c in g.findall("chain")],
    }
print(json.dumps({
    "groups": groups,
    "end_effectors": [{"name": e.get("name"), "parent_link": e.get("parent_link"),
                       "parent_group": e.get("parent_group")}
                      for e in root.findall("end_effector")],
    "virtual_joints": [{"parent_frame": v.get("parent_frame"), "child_link": v.get("child_link")}
                       for v in root.findall("virtual_joint")],
}))
"""

# A robot description is latched (TRANSIENT_LOCAL): echo it with that
# durability, else ask the node that holds it as a parameter.
_DESCRIPTION = ("(ros2 topic echo --once --qos-durability transient_local "
                "--qos-reliability reliable --field data {topic} 2>/dev/null "
                "|| ros2 param get {node} {param} 2>/dev/null "
                "| sed 's/^String value is: //')")


def read_graph(run: Run) -> dict:
    """The graph as seen from where ``run`` executes: topics, actions and
    services with their types, the joints each trajectory and gripper
    controller drives, and the URDF and SRDF facts when they are served."""
    def typed(cmd: str) -> dict[str, str]:
        out = {}
        for line in run(cmd).splitlines():
            name, _, rest = line.strip().partition(" [")
            if name:
                out[name] = rest.rstrip("]").split(", ")[0] if rest else ""
        return out

    graph = {"topics": typed("ros2 topic list -t"),
             "actions": typed("ros2 action list -t"),
             "services": [x.strip() for x in run("ros2 service list").splitlines() if x.strip()],
             "urdf": None, "srdf": None, "controllers": {}}
    for key, topic, node, param, script in (
            ("urdf", "/robot_description", "/robot_state_publisher", "robot_description",
             _URDF_FACTS),
            ("srdf", "/robot_description_semantic", "/move_group", "robot_description_semantic",
             _SRDF_FACTS)):
        cmd = (_DESCRIPTION.format(topic=topic, node=node, param=param)
               + " | python3 -c " + _shell_quote(script))
        try:
            out = run(cmd).strip()
            graph[key] = json.loads(out) if out.startswith("{") else None
        except Exception:  # noqa: BLE001  no description is a finding, not a failure
            graph[key] = None
    # The joints a controller drives are its own parameters: an action
    # /<controller>/follow_joint_trajectory is served by node /<controller>.
    for action, kind in graph["actions"].items():
        if kind.endswith("FollowJointTrajectory") or kind.endswith("GripperCommand"):
            node = action.rsplit("/", 1)[0]
            param = "joints" if kind.endswith("FollowJointTrajectory") else "joint"
            try:
                graph["controllers"][action] = _param_value(
                    run(f"ros2 param get {node} {param} 2>/dev/null || true"))
            except Exception:  # noqa: BLE001  a controller without the parameter
                graph["controllers"][action] = None
    return graph


def _param_value(out: str):
    """`ros2 param get` output -> the value: a string or a list of them."""
    out = out.strip()
    for prefix in ("String values are: ", "String value is: "):
        if out.startswith(prefix):
            value = out[len(prefix):].strip()
            return ast.literal_eval(value) if prefix.startswith("String values") else value
    return None


def _shell_quote(s: str) -> str:
    return "'" + s.replace("'", "'\\''") + "'"


def _of_type(names: dict[str, str], suffix: str) -> list[str]:
    return sorted(n for n, t in names.items() if t.endswith(suffix))


def draft_profile(graph: dict, discovery: dict, *, ros_distro: str | None = None) -> str:
    """A robot profile in YAML, filled from the graph where the graph
    knows; ``TODO`` marks what a person must decide."""
    u = graph.get("urdf") or {}
    srdf = graph.get("srdf") or {}
    joints = {j["name"]: j for j in u.get("joints", [])}
    actions, topics, services = graph["actions"], graph["topics"], graph["services"]
    controllers = graph.get("controllers", {})
    trajectories = _of_type(actions, "FollowJointTrajectory")
    grippers = _of_type(actions, "GripperCommand")
    twists = _of_type(topics, "TwistStamped")
    wrenches = _of_type(topics, "WrenchStamped")
    arm_joints = list(controllers.get(trajectories[0]) or []) if trajectories else []
    moveit = "/move_action" in actions or any(s.endswith("compute_ik") for s in services)
    group, hand = _planning_group(srdf, arm_joints)

    y = []
    y.append("# Drafted by `openrua probe` from the live graph. Lines marked TODO need")
    y.append("# a person and fail validation until filled in; everything else was read")
    y.append("# from the robot. Delete any port the robot does not actually serve: trial")
    y.append("# preflight checks the listed ports, and the workspace documents them for")
    y.append("# the agent.")
    y.append("# type: panda   # TODO if this is an instance of a bundled robot type (openrua")
    y.append("#               # robots): name it and delete the facts below that it supplies")
    y.append("machine:")
    y.append("  backend:")
    y.append("    kind: real")
    if ros_distro is None:
        y.append("    ros_distro: jazzy   # TODO: the distro the robot runs; the sandbox image follows from it")
    else:
        y.append(f"    ros_distro: {json.dumps(ros_distro)}")
    y.append("    # launch: <command that starts the driver>   # TODO if the graph is not already up")
    y.append("    discovery:")
    for k, v in discovery.items():
        y.append(f"      {k}: {json.dumps(v)}")
    y.append("  robot:")
    y.append(f"    model: {u.get('name') or 'TODO'}   # TODO: the model as a person would name it")
    y.append("    description: TODO   # one line: arm, gripper, base")
    y.append("  frames:")
    y.append("    world: TODO   # the fixed frame points are expressed in (often the arm's root)")
    y.append(f"    base: {u.get('root') or 'TODO'}")
    y.append(f"    hand: {hand or 'TODO'}" + ("" if hand else "   # the link IK targets"))
    if arm_joints:
        y.append("  arm:   # the joints " + trajectories[0] + " drives, in its order")
        y.append("    joints: [" + ", ".join(arm_joints) + "]")
        lim = ", ".join(f"[{_num(joints.get(j, {}).get('lower'))}, "
                        f"{_num(joints.get(j, {}).get('upper'))}]" for j in arm_joints)
        y.append(f"    limits_rad: [{lim}]")
    else:
        y.append("  arm:   # TODO: no trajectory controller named its joints")
        y.append("    joints: []")
        y.append("    limits_rad: []")
    if grippers:
        joint = controllers.get(grippers[0])
        j = joints.get(joint or "", {})
        y.append("  gripper:   # the GripperCommand interface, then what the driver does with a goal")
        y.append(f"    joint: {joint or 'TODO'}")
        travel = (f"; the joint is {j['type']} and travels {_num(j.get('lower'))} .. "
                  f"{_num(j.get('upper'))}" if j else "")
        y.append(f"    unit: TODO   # m or rad: the GripperCommand position's unit{travel}")
        y.append("    open: TODO   # the position that opens it fully")
        y.append("    closed: TODO   # the position that closes it fully")
        y.append("    max_effort: TODO   # the max_effort a goal asks for")
        y.append("    holds_partway: TODO   # true if a position between open and closed is held")
        y.append("    effort_honoured: TODO   # true if max_effort limits the grip")
    if moveit:
        y.append("  planning:")
        y.append("    moveit: true")
        y.append(f"    move_action: {'/move_action' if '/move_action' in actions else 'TODO'}")
        ik = next((s for s in services if s.endswith("compute_ik")), None)
        y.append(f"    ik_service: {ik or 'TODO'}")
        y.append(f"    group: {group or 'TODO'}" + ("" if group else "   # the MoveIt planning group"))
        virtual = (srdf.get("virtual_joints") or [{}])[0].get("parent_frame")
        y.append(f"    planning_frame: {virtual or u.get('root') or 'TODO'}")
    ports = {"trajectory": trajectories[0] if trajectories else None,
             "gripper": grippers[0] if grippers else None,
             "twist": twists[0] if len(twists) == 1 else None,
             "wrench": wrenches[0] if len(wrenches) == 1 else None}
    found = {k: v for k, v in ports.items() if v}
    y.append("  ports:" if found else "  ports: {}   # TODO: none of the usual ports found on the graph")
    for k, v in found.items():
        y.append(f"    {k}: {v}")
    for k, many in (("trajectory", trajectories), ("gripper", grippers),
                    ("twist", twists), ("wrench", wrenches)):
        if len(many) > 1 and k in ("twist", "wrench"):
            y.append(f"    # {k}: TODO one of {', '.join(many)}")
        elif len(many) > 1:
            y.append(f"    # also on the graph ({k}): {', '.join(many[1:])}")
        elif not many:
            y.append(f"    # {k}: null   # not found on the graph")
    y.extend(_camera_lines(topics))
    y.append("  workspace_template: workspace")
    return "\n".join(y) + "\n"


def _planning_group(srdf: dict, arm_joints: list[str]) -> tuple[str | None, str | None]:
    """The SRDF group that covers the arm's joints, and the link IK
    targets: an end effector's parent link on that group, else the tip
    of the group's chain."""
    groups = srdf.get("groups") or {}
    group = next((g for g, d in groups.items()
                  if arm_joints and set(arm_joints) <= set(d.get("joints") or [])), None)
    group = group or next((g for g, d in groups.items() if d.get("chains")), None)
    if group is None:
        return None, None
    hand = next((e["parent_link"] for e in srdf.get("end_effectors") or []
                 if e.get("parent_group") == group and e.get("parent_link")), None)
    chains = groups[group].get("chains") or []
    return group, hand or (chains[0][1] if chains else None)


def _camera_lines(topics: dict[str, str]) -> list[str]:
    """One draft entry per camera: a CameraInfo topic names a camera; its
    type file (topics, depth encoding) and mount are the owner's."""
    infos = _of_type(topics, "CameraInfo")
    images = _of_type(topics, "sensor_msgs/msg/Image")
    if not infos:
        return ["  cameras: {list: []}   # no CameraInfo topic on the graph"]
    out = ["  cameras:",
           "    list:   # TODO each camera's type: a file with its topics ({name} = the",
           "            # camera's name) and depth encoding; see docs/your-own-robot.md"]
    seen = set()
    for info in infos:
        name = info.strip("/").split("/")[0]
        if name in seen:
            continue
        seen.add(name)
        streams = [t for t in images if t.startswith(f"/{name}/")]
        out.append(f"      - name: {name}")
        out.append("        type: TODO   # streams: " + ", ".join(streams + [info]))
        out.append("        mount: TODO   # wrist or fixed")
    return out


def _num(v) -> str:
    return "null" if v is None else f"{float(v):.3f}".rstrip("0").rstrip(".") if float(v) != int(float(v)) else f"{float(v):.1f}"
