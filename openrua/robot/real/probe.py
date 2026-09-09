"""Read a live ROS 2 graph and draft a robot profile from it.

What a person would otherwise type into a profile by hand is mostly
already on the graph: the topics, actions and services the robot serves,
and the joint names, limits and frames in ``/robot_description``. This
module reads them through a caller-supplied ``run`` (a shell executed
where the graph is visible, the sandbox) and prints a profile draft with
``# TODO`` on the few facts only a person knows.
"""

from __future__ import annotations

import json
from typing import Callable

Run = Callable[[str], str]

# The python that turns the published URDF into the facts a profile
# carries; runs where urdfdom-py is installed (the sandbox image).
_URDF_FACTS = r"""
import json, sys
from urdf_parser_py.urdf import URDF
r = URDF.from_xml_string(sys.stdin.read())
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


def read_graph(run: Run) -> dict:
    """The graph as seen from where ``run`` executes: topic, action and
    service names, and the URDF facts when /robot_description is served."""
    def lines(cmd: str) -> list[str]:
        return [x.strip() for x in run(cmd).splitlines() if x.strip()]
    graph = {"topics": lines("ros2 topic list"),
             "actions": lines("ros2 action list"),
             "services": lines("ros2 service list"),
             "urdf": None}
    urdf_cmd = ("(ros2 topic echo --once --field data /robot_description 2>/dev/null "
                "|| ros2 param get /robot_state_publisher robot_description 2>/dev/null "
                "| sed 's/^String value is: //') | python3 -c " + _shell_quote(_URDF_FACTS))
    try:
        out = run(urdf_cmd).strip()
        graph["urdf"] = json.loads(out) if out.startswith("{") else None
    except Exception:  # noqa: BLE001  no URDF is a finding, not a failure
        graph["urdf"] = None
    return graph


def _shell_quote(s: str) -> str:
    return "'" + s.replace("'", "'\\''") + "'"


def _pick(names: list[str], *needles: str) -> str | None:
    for n in needles:
        for name in names:
            if n in name:
                return name
    return None


def draft_profile(graph: dict, discovery: dict) -> str:
    """A robot profile in YAML, filled from the graph where the graph
    knows; ``# TODO`` marks what a person must decide."""
    u = graph.get("urdf") or {}
    joints = u.get("joints", [])
    arm = [j for j in joints if j["type"] in ("revolute", "continuous")]
    gripper = [j for j in joints if j["type"] == "prismatic"]
    actions, topics, services = graph["actions"], graph["topics"], graph["services"]
    ports = {
        "trajectory": _pick(actions, "follow_joint_trajectory"),
        "gripper": _pick(actions, "gripper_cmd", "gripper_action", "gripper"),
        "twist": _pick(topics, "delta_twist_cmds", "twist_cmd", "servo"),
        "wrench": _pick(topics, "wrench", "external_wrench"),
    }
    cameras = sorted({t.rsplit("/color/image_raw", 1)[0].rsplit("/", 1)[-1]
                      for t in topics if t.endswith("/color/image_raw")})
    moveit = bool(_pick(actions, "move_action")) or bool(_pick(services, "compute_ik"))
    y = []
    y.append("# Drafted by `openrua probe` from the live graph. Lines marked TODO need")
    y.append("# a person; everything else was read from the robot. Delete any port the")
    y.append("# robot does not actually serve: every port listed is a promise preflight")
    y.append("# checks and the agent is told about.")
    y.append("# type: panda   # TODO if this is an instance of a bundled robot type (openrua")
    y.append("#               # robots): name it and delete the facts below that it supplies")
    y.append("machine:")
    y.append("  backend:")
    y.append("    kind: real")
    y.append("    ros_distro: jazzy   # TODO: the distro the robot runs; the sandbox image follows from it")
    y.append("    # launch: <command that starts the driver>   # TODO if the graph is not already up")
    y.append("    discovery:")
    for k, v in discovery.items():
        y.append(f"      {k}: {json.dumps(v)}")
    y.append("  robot:")
    y.append(f"    model: {u.get('name') or 'TODO'}   # TODO: the model as a person would name it")
    y.append("    description: TODO   # one line: arm, gripper, base")
    y.append("  frames:")
    y.append("    world: world   # TODO: the fixed frame the robot plans in")
    y.append(f"    base: {u.get('root') or 'TODO'}")
    hand = _pick(u.get("links", []), "tool0", "hand", "tcp", "ee_link", "gripper")
    y.append(f"    hand: {hand or 'TODO'}")
    if arm:
        y.append("  arm:")
        y.append("    joints: [" + ", ".join(j["name"] for j in arm) + "]")
        lim = ", ".join(f"[{_num(j['lower'])}, {_num(j['upper'])}]" for j in arm)
        y.append(f"    limits_rad: [{lim}]")
    else:
        y.append("  arm:   # TODO: no revolute joints found in /robot_description")
        y.append("    joints: []")
        y.append("    limits_rad: []")
    if gripper:
        g = gripper[0]
        y.append("  gripper:")
        y.append(f"    open_m: {_num(g['upper'])}")
        y.append(f"    closed_m: {_num(g['lower'])}")
        y.append("    max_effort: 30.0   # TODO")
        y.append("    stops_at: [open, closed]")
    if moveit:
        y.append("  planning:")
        y.append("    moveit: true")
        y.append(f"    move_action: {_pick(actions, 'move_action') or '/move_action'}")
        y.append(f"    ik_service: {_pick(services, 'compute_ik') or '/compute_ik'}")
        y.append("    group: TODO   # the MoveIt planning group")
        y.append(f"    planning_frame: {u.get('root') or 'TODO'}")
    found = {k: v for k, v in ports.items() if v}
    y.append("  ports:" if found else "  ports: {}   # TODO: none of the usual ports found on the graph")
    for k, v in found.items():
        y.append(f"    {k}: {v}")
    for k in ports:
        if k not in found:
            y.append(f"    # {k}: null   # not found on the graph")
    if cameras:
        y.append("  cameras:")
        y.append("    list: [" + ", ".join(cameras) + "]")
    y.append("  workspace_template: workspace")
    return "\n".join(y) + "\n"


def _num(v) -> str:
    return "null" if v is None else f"{float(v):.3f}".rstrip("0").rstrip(".") if float(v) != int(float(v)) else f"{float(v):.1f}"
