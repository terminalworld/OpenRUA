#!/usr/bin/env python3
"""Move the hand to a planning-frame pose through IK and one trajectory.

Usage: python3 tools/action/ik_move.py <x> <y> <z> <qx> <qy> <qz> <qw> \
           [seconds=4] [--at tcp|hand]
Pose is where the HAND frame goes; --at tcp aims the fingertip point
instead, using machine.yaml hand.tcp_offset_m along the hand's +Z axis.
Coordinates are in machine.yaml planning.planning_frame, not necessarily
the world frame. IK failures
exit loudly with the MoveIt error code; no solution means NO MOTION
happened. Approach direction, grasp logic, verification stay yours.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import yaml


def manifest() -> dict:
    root = Path(__file__).resolve().parents[2]
    return yaml.safe_load((root / "machine.yaml").read_text())


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("x", "y", "z", "qx", "qy", "qz", "qw"):
        parser.add_argument(name, type=float)
    parser.add_argument("seconds", type=float, nargs="?", default=4.0,
                        help="trajectory duration in seconds (default: 4)")
    parser.add_argument("--at", choices=("tcp", "hand"), default="hand",
                        help="point to place at the target pose (default: hand)")
    return parser.parse_intermixed_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    from moveit_msgs.srv import GetPositionIK
    from sensor_msgs.msg import JointState
    import rclpy

    x, y, z, qx, qy, qz, qw = (getattr(args, name)
                               for name in ("x", "y", "z", "qx", "qy", "qz", "qw"))
    seconds = args.seconds
    at_tcp = args.at == "tcp"

    m = manifest()
    traj_entry = next(a for a in m["actuators"]
                      if a["kind"] == "joint_trajectory")
    planning = m.get("planning", {})
    if at_tcp:
        off = float(m.get("hand", {}).get("tcp_offset_m", 0.0))
        # shift the target back along the hand's approach (+Z of the
        # hand frame, expressed in the planning frame via the quaternion)
        R = _quat_to_R(qx, qy, qz, qw)
        x, y, z = np.array([x, y, z]) - off * R[:, 2]

    rclpy.init()
    node = rclpy.create_node("ik_move")
    js = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda msg: js.setdefault("m", msg), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while "m" not in js and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if "m" not in js:
        raise SystemExit("no /joint_states")

    cli = node.create_client(GetPositionIK, planning.get("ik_service",
                                                         "/compute_ik"))
    if not cli.wait_for_service(timeout_sec=10):
        raise SystemExit("IK service unavailable")
    req = GetPositionIK.Request()
    req.ik_request.group_name = planning.get("group", "panda_arm")
    # machine fact: leave frame_id empty; poses are interpreted in the
    # planner's model frame (the arm base), see machine.yaml planning
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y = qx, qy
    p.orientation.z, p.orientation.w = qz, qw
    # seed with ARM joints only: a full /joint_states (finger joints
    # included) can hang move_group's IK service indefinitely
    seed = JointState()
    arm = set(traj_entry["joints"])
    for n_, p_ in zip(js["m"].name, js["m"].position):
        if n_ in arm:
            seed.name.append(n_)
            seed.position.append(p_)
    req.ik_request.robot_state.joint_state = seed
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    res = fut.result()
    if res is None:
        raise SystemExit("IK service did not answer within 60s; no motion "
                         "sent (service load, not necessarily unreachable)")
    if res.error_code.val != 1:
        raise SystemExit(f"IK FAILED (error_code={res.error_code.val}) "
                         "; no motion sent")

    sol = dict(zip(res.solution.joint_state.name,
                   res.solution.joint_state.position))
    joints = traj_entry["joints"]
    positions = ",".join(f"{sol[j]:.6f}" for j in joints)
    rclpy.shutdown()
    # execute through the standard trajectory tool (same act path you
    # would use by hand)
    import subprocess
    r = subprocess.run([sys.executable,
                        str(Path(__file__).parent / "fjt_send.py"),
                        positions, str(seconds)])
    sys.exit(r.returncode)


def _quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


if __name__ == "__main__":
    main()
