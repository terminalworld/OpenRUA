"""The agent's workspace: template seeding + machine manifest.

Everything the agent is given beyond the robot's native surface lives
in the workspace: starter docs, tools, and the generated machine.yaml
the agent finds in /workspace. This module owns what the workspace is
(content and generation); when and where it is seeded per trial is the
runner's call.
"""

from __future__ import annotations

import argparse
import re
import hashlib
import shutil
from pathlib import Path

import yaml

from openrua.sandbox import SandboxError

_HERE = Path(__file__).resolve().parent


def template_dir(cfg: dict) -> Path | None:
    """The configured template tree (a directory name under this package),
    or None when the config seeds no workspace (cold-start arm)."""
    name = cfg.get("machine", {}).get("workspace_template")
    return (_HERE / name) if name else None


def seed(cfg: dict, dest: Path) -> None:
    """Fresh per-trial workspace: template copy + generated machine.yaml.
    No-op when no template is configured."""
    root = template_dir(cfg)
    if not root:
        return
    if not root.is_dir():
        # A config typo must not surface as a copytree traceback.
        raise SandboxError(
            f"workspace_template {root.name!r} not found under the "
            f"sandbox package ({root.parent})")
    shutil.copytree(root, dest, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns("__pycache__"))
    write_machine_manifest(cfg, dest / "machine.yaml")


def template_hash(cfg: dict) -> str:
    """Deterministic sha256 of the template tree, recorded with every
    trial like the prompt's; "" when no template."""
    root = template_dir(cfg)
    if not root:
        return ""
    h = hashlib.sha256()
    for p in sorted(root.rglob("*")):
        # Byte trash (interpreter caches) must never move the method
        # artifact's hash: importing a tool (preflight smoke) would
        # otherwise mutate provenance.
        if "__pycache__" in p.parts:
            continue
        if p.is_file():
            # Delimited per-file records: bare path+content
            # concatenation is collision-constructible
            # (file "ab"+"c" == file "a"+"bc"); a NUL after the path and
            # a content digest give every file an unambiguous record.
            h.update(str(p.relative_to(root)).encode() + b"\0")
            h.update(hashlib.sha256(p.read_bytes()).digest())
    return h.hexdigest()


# What each manifest key means, written onto the manifest itself rather
# than into the generic docs: a value and its definition belong in one
# place, and which of these keys even appear is a per-machine fact.
# Keys that repeat on every entry (kind/port/type) are explained once in
# the header instead of on all twenty lines.
_HEADER = """\
# This machine's manifest: an index of its ROS graph, written when the
# machine was assembled. Each entry under actuators/sensors is one port.
#   kind  what the port is for; docs/ has a how-to section per kind
#   port  the ROS name to address
#   type  the message or action type; `ros2 interface show <type>`
#         prints its fields
# The values below are this robot's configuration as its integrator
# wrote it; the docs are generic. Like any robot config it can drift from
# the running system: where the live graph disagrees (topic names,
# message types, frames, limits, sensor behaviour), the live graph is
# authoritative. Discrepancies go in machine-discrepancies.yaml next to
# this file (key, declared, observed, evidence) for review and merging
# into the config; this file is regenerated from the config at every
# bring-up.
"""

_FIELD_NOTES = {
    "joints": "the joints this port drives, in the order its messages\nexpect them",
    "limits_rad": "travel per joint as [min, max], radians",
    "frame": "the TF frame this port's messages are expressed in",
    "joint": "the joint whose /joint_states position reports the\ngripper's",
    "unit": "unit of the GripperCommand position and of that joint",
    "open": "the position that opens the gripper fully",
    "closed": "the position that closes it fully; which way closes\nfollows from open and closed",
    "max_effort": "the GripperCommand max_effort a goal asks for unless\ntold otherwise",
    "holds_partway": "true: a position between open and closed is held;\nfalse: it is read as open or closed, whichever is nearer",
    "effort_honoured": "true: max_effort limits the grip; false: it is ignored",
    "name": "the camera's name on this machine",
    "model": "what it is, as a person would name it",
    "mount": "wrist: moves with the arm; fixed: stands in the scene",
    "color": "sensor_msgs/Image color stream",
    "info": "sensor_msgs/CameraInfo of the color stream: intrinsics k,\nand header.frame_id, the optical frame its pixels are in",
    "depth": "sensor_msgs/Image depth stream",
    "depth_encoding": "32FC1 = metres, 16UC1 = millimetres (ROS REP 118)",
    "depth_aligned": "true: depth pixels line up with the color image, so the\ncolor intrinsics and frame serve both",
    "discover": "shell command that lists what exists right now",
    "moveit": "whether a MoveIt planner is running on this machine",
    "move_action": "MoveIt's plan-and-execute action",
    "ik_service": "inverse-kinematics service",
    "tcp_offset_m": "hand frame -> the point between the fingertips, along\nthe hand's approach axis, metres",
    "group": "the MoveIt planning group covering the arm",
    "planning_frame": "the frame MoveIt plans in",
    "arm": "which arm of a multi-arm machine this port belongs to",
    "base_frame": "this arm's root link",
    "hand_frame": "this arm's hand frame, the link its IK targets",
    "simulation": "how this simulated machine behaves where a real one\nwould not",
    "clock": "paused: the world moves only while a command executes, so\nwaiting changes nothing, objects settle only as commands run,\nand seconds of motion can take minutes of wall time under\nload (a client timeout is not a failed motion); free_running:\nthe world runs with the wall clock",
    "camera_render": "on_demand: a camera renders only while something\nsubscribes, so its topics can appear late; always: every\nframe is rendered",
    "camera_rate_hz": "frames each camera publishes per wall-clock second",
    "trajectory_goal_tolerance_rad": "a FollowJointTrajectory goal succeeds when every\njoint ends within this of its last point, radians",
    "gripper_settle_ticks": "control ticks a gripper goal advances the world",
    "base_command": "queue: each base Twist drives one control tick, then\nstops; latest: a new Twist replaces the one not yet applied",
    "base_queue": "most base ticks that may wait; a Twist beyond is dropped",
    "notes": "observations about this machine that no field can say,\nby the kind they concern",
    "drive": "the base's drive geometry",
}


def _annotate(text: str) -> str:
    """Append the key's definition as a YAML comment on its line."""
    out = []
    for line in text.splitlines():
        m = re.match(r"^(\s*(?:- )?)([A-Za-z_]+):(\s|$)", line)
        note = _FIELD_NOTES.get(m.group(2)) if m else None
        if not note:
            out.append(line)
            continue
        pad = " " * len(m.group(1))
        first, *rest = note.split("\n")
        out.append(f"{line}  # {first}")
        out += [f"{pad}  # {r}" for r in rest]
    return "\n".join(out) + "\n"


def _has_joints(m: dict) -> bool:
    """Whether the machine has anything that reports on /joint_states: an
    arm, a gripper, or a mobile base."""
    return bool(m.get("base") or any(a.get("joints") or a.get("gripper")
                                      for a in m.get("arms", [])))


def _camera_entries(cams: dict) -> list[dict]:
    """One sensor entry per declared camera, its topics filled in from
    its type (machine.cameras.types, read by the loader). A scene that
    decides its own cameras (list: null) gets the topic pattern and a
    command that lists them."""
    types, entries = cams.get("types") or {}, cams.get("list")

    def streams(t: dict, topics: dict, name: str) -> dict:
        out = {k: topics[k].replace("{name}", name)
               for k in ("color", "info", "depth") if topics.get(k)}
        if topics.get("depth") and t.get("depth"):
            out["depth_encoding"] = t["depth"]["encoding"]
            out["depth_aligned"] = t["depth"]["aligned_to_color"]
        return out

    if entries is None:
        if "openrua-sim" not in types:
            raise ValueError("machine.cameras.types is missing: write the manifest from "
                             "a resolved config (openrua.config.compose)")
        t = types["openrua-sim"]
        suffix = t["topics"]["color"].split("{name}")[-1]
        return [{"kind": "camera_set", **streams(t, t["topics"], "<name>"),
                 "discover": f"ros2 topic list | grep {suffix}"}]
    out = []
    for e in entries:
        e = {"name": e, "type": "openrua-sim"} if isinstance(e, str) else e
        t = types[e["type"]]
        entry = {"kind": "camera", "name": e["name"], "model": t["model"]}
        if e.get("mount"):
            entry["mount"] = e["mount"]
        entry.update(streams(t, {**t["topics"], **(e.get("topics") or {})}, e["name"]))
        out.append(entry)
    return out


def _simulation(cfg: dict, manifest: dict) -> dict:
    """The values a simulated machine's bridge acts on and a real robot
    does not have, read from the same config that starts the bridge."""
    m, protocol = cfg.get("machine", {}), cfg.get("protocol", {})
    kinds = {e["kind"] for e in manifest["actuators"]}
    sim = {"clock": protocol.get("clock", {}).get("mode", "paused")}
    if any(s["kind"] in ("camera", "camera_set") for s in manifest["sensors"]):
        sim["camera_render"] = m.get("cameras", {}).get("render_mode", "on_demand")
        sim["camera_rate_hz"] = m.get("cameras", {}).get("rate_hz", 2.0)
    control = m.get("control", {})
    if "joint_trajectory" in kinds:
        sim["trajectory_goal_tolerance_rad"] = control.get("trajectory", {}).get(
            "goal_tolerance_rad", 0.05)
    if "gripper" in kinds:
        sim["gripper_settle_ticks"] = control.get("gripper", {}).get("settle_steps", 10)
    if "base_twist" in kinds:
        sim["base_command"] = protocol.get("base_cmd_policy", "queue")
        if sim["base_command"] == "queue":
            sim["base_queue"] = protocol.get("base_cmd_queue", 4)
    return sim


def _notes(m: dict, manifest: dict) -> dict:
    """The machine's notes for the kinds it actually has: an observation
    about a base or a gripper means nothing on a machine without one."""
    present = {e["kind"] for e in manifest["actuators"] + manifest["sensors"]}
    present = {("camera" if k == "camera_set" else k) for k in present} | {"timing"}
    if (m.get("planning") or {}).get("moveit"):
        present.add("planning")
    return {k: v for k, v in (m.get("notes") or {}).items() if k in present and v}


def write_machine_manifest(cfg: dict, out: Path) -> None:
    """Generate the workspace machine.yaml from the resolved config.

    The manifest is a curated index of the ROS graph; typed port entries
    grouped by capability kind (open lists: a dual-arm robot is two
    joint_trajectory entries; a mobile base adds a base_twist kind). The
    generic docs/tools read it; benchmark specifics live only in the
    config and the manifest derives from the same config that assembles
    the bridge.
    """
    m = cfg.get("machine", {})
    ports = m.get("ports", {})
    # Per-arm entries over the normalized view (run.normalize_arms); a
    # single-arm machine has one unlabeled arm and the entries carry no
    # "arm" column, exactly the historical manifest.
    arm_actuators, arm_sensors = [], []
    for arm in m.get("arms", []):
        tag = {"arm": arm["label"]} if arm.get("label") else {}
        aports = arm.get("ports", {})
        arm_actuators += [
            {"kind": "joint_trajectory", "port": aports.get("trajectory"),
             "type": "control_msgs/action/FollowJointTrajectory",
             "joints": arm.get("joints", []),
             "limits_rad": arm.get("limits_rad", []), **tag,
             # Each arm's own frames; one arm's are the machine's frames.
             **({"base_frame": arm.get("base_frame"), "hand_frame": arm.get("hand_frame")}
                if tag else {})},
            {"kind": "cartesian_twist", "port": aports.get("twist"),
             "type": "geometry_msgs/msg/TwistStamped",
             "frame": arm.get("base_frame"), **tag},
            # The typed columns first; the gripper's facts after them
            # (a config "type" must not clobber the ROS message type).
            {"kind": "gripper", "port": aports.get("gripper"),
             "type": "control_msgs/action/GripperCommand", **tag,
             **{k: v for k, v in (arm.get("gripper") or {}).items()
                if k not in ("kind", "port", "type", "arm")}},
        ]
        arm_sensors.append(
            {"kind": "wrench", "port": aports.get("wrench"),
             "type": "geometry_msgs/msg/WrenchStamped", **tag})
    camera_sensors = _camera_entries(m.get("cameras") or {})
    manifest = {
        "schema": "openrua/machine-manifest v1",
        "robot": m.get("robot", {}),
        "frames": m.get("frames", {}),
        "actuators": arm_actuators + [
            {**{("drive" if k == "type" else k): v
                for k, v in m.get("base", {}).items()
                if k not in ("body", "cmd_vel", "odom")},
             "kind": "base_twist", "port": ports.get("base_twist"),
             "type": "geometry_msgs/msg/Twist"},
        ],
        "sensors": [
            # joint states come from whatever has joints; a machine of
            # cameras alone publishes none
            {"kind": "joint_state",
             "port": "/joint_states" if _has_joints(m) else None,
             "type": "sensor_msgs/msg/JointState"},
            *arm_sensors,
            {"kind": "odometry", "port": ports.get("odom"),
             "type": "nav_msgs/msg/Odometry",
             "frame": m.get("base", {}).get("frame")},
            *camera_sensors,
        ],
        "planning": m.get("planning", {}),
        "hand": m.get("hand", {}),
    }
    manifest["actuators"] = [a for a in manifest["actuators"] if a.get("port")]
    manifest["sensors"] = [s for s in manifest["sensors"]
                           if s.get("port") or s.get("color")]
    if m.get("backend", {}).get("kind") == "sim":
        manifest["simulation"] = _simulation(cfg, manifest)
    notes = _notes(m, manifest)
    if notes:
        manifest["notes"] = notes
    out.write_text(_HEADER + _annotate(
        yaml.safe_dump(manifest, sort_keys=False, width=1000)))


def main() -> int:  # standalone: seed a workspace / print the hash
    ap = argparse.ArgumentParser(
        description="Seed an agent workspace from an resolved config "
                    "(or print the template hash).")
    ap.add_argument("--config", required=True,
                    help="resolved config yaml path (the suite view)")
    ap.add_argument("--dest", default=None,
                    help="workspace dir to seed; omit to only print the hash")
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.config).read_text())
    try:
        if args.dest:
            seed(cfg, Path(args.dest))
            print(f"seeded {args.dest} (template sha256 {template_hash(cfg)})")
        else:
            print(template_hash(cfg))
    except SandboxError as e:
        raise SystemExit(str(e))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
