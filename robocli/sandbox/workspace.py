"""The agent's workspace: template seeding + machine manifest.

The workspace is the skin layer's SOLE carrier (design-decisions
2026-08-10): starter docs, tools, and the generated machine.yaml the
agent finds in /workspace. This module owns what the workspace IS;
content and generation; when/where it is seeded per trial is the
evaluator side's call (robocli/run.py).
"""

from __future__ import annotations

import hashlib
import shutil
from pathlib import Path

import yaml

from robocli.sandbox import SandboxError

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
    """Deterministic sha256 of the template tree (prompt-equal provenance
    status, design-decisions 2026-08-10); "" when no template."""
    root = template_dir(cfg)
    if not root:
        return ""
    h = hashlib.sha256()
    for p in sorted(root.rglob("*")):
        # Byte trash (interpreter caches) must never move the method
        # artifact's hash: importing a tool (precheck smoke) would
        # otherwise mutate provenance.
        if "__pycache__" in p.parts:
            continue
        if p.is_file():
            # Delimited per-file records (review 2026-08-15 M1): bare
            # path+content concatenation is collision-constructible
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
# The docs are generic; the facts below are this machine's and win.
"""

_FIELD_NOTES = {
    "joints": "the joints this port drives, in the order its messages\nexpect them",
    "limits_rad": "travel per joint as [min, max], radians",
    "frame": "the TF frame this port's messages are expressed in",
    "open_m": "fingers fully open: position of ONE finger, metres\n(the gap between the fingers is twice this)",
    "closed_m": "fingers fully closed: position of ONE finger, metres",
    "max_effort": "force ceiling a GripperCommand goal may ask for,\nnewtons. NOT honoured by this machine - see stops_at",
    "stops_at": "the only positions this gripper comes to rest at. A\ncommanded width is read as open-or-closed; there is no\nholding it part-way, and no force limit",
    "topics": "topic name pattern, one set per camera",
    "discover": "shell command that lists what exists right now",
    "moveit": "whether a MoveIt planner is running on this machine",
    "move_action": "MoveIt's plan-and-execute action",
    "ik_service": "inverse-kinematics service (leave frame_id empty)",
    "group": "the MoveIt planning group covering the arm",
    "planning_frame": "the frame MoveIt plans in",
    "arm": "which arm of a multi-arm machine this port belongs to",
    "drive": "the base's drive geometry",
}


def _annotate(text: str) -> str:
    """Append the key's definition as a YAML comment on its line."""
    import re

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


def write_machine_manifest(cfg: dict, out: Path) -> None:
    """Generate the workspace machine.yaml from the assembly config.

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
             "limits_rad": arm.get("limits_rad", []), **tag},
            {"kind": "cartesian_twist", "port": aports.get("twist"),
             "type": "geometry_msgs/msg/TwistStamped",
             "frame": arm.get("base_frame"), **tag},
            # Config extras spread FIRST so the typed columns always
            # win (review 2026-08-15 M2: robocasa's machine.base.type
            # "holonomic" was clobbering the ROS message type); a
            # config "type" survives as "drive".
            {**{("drive" if k == "type" else k): v
                for k, v in (arm.get("gripper") or {}).items()},
             "kind": "gripper", "port": aports.get("gripper"),
             "type": "control_msgs/action/GripperCommand", **tag},
        ]
        arm_sensors.append(
            {"kind": "wrench", "port": aports.get("wrench"),
             "type": "geometry_msgs/msg/WrenchStamped", **tag})
    manifest = {
        "schema": "robocli/machine-manifest v1",
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
            {"kind": "joint_state", "port": "/joint_states",
             "type": "sensor_msgs/msg/JointState"},
            *arm_sensors,
            {"kind": "odometry", "port": ports.get("odom"),
             "type": "nav_msgs/msg/Odometry",
             "frame": m.get("base", {}).get("frame")},
            {"kind": "camera_set",
             "topics": "/<name>/color/image_raw, /<name>/depth/image_raw, "
                       "/<name>/color/camera_info",
             "discover": "ros2 topic list | grep image_raw"},
        ],
        "planning": m.get("planning", {}),
        "hand": m.get("hand", {}),
    }
    manifest["actuators"] = [a for a in manifest["actuators"] if a.get("port")]
    manifest["sensors"] = [s for s in manifest["sensors"]
                           if s.get("port") or s.get("topics")]
    out.write_text(_HEADER + _annotate(
        yaml.safe_dump(manifest, sort_keys=False, width=1000)))


def main() -> int:  # standalone: seed a workspace / print the hash
    import argparse

    ap = argparse.ArgumentParser(
        description="Seed an agent workspace from an assembly config "
                    "(or print the template hash).")
    ap.add_argument("--config", required=True,
                    help="RESOLVED assembly yaml path (the suite view)")
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
