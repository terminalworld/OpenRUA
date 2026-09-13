#!/usr/bin/env python3
"""Probe IK feasibility for a list of TCP poses (base frame) without moving.

Usage: python3 ik_probe.py "x y z qx qy qz qw" ["x y z qx qy qz qw" ...]
Prints the joint solution (manifest order) or the error code per pose.
"""
import sys

import numpy as np
import rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

sys.path.insert(0, "/workspace/tools/action")
from ik_move import _quat_to_R  # noqa: E402

TCP = 0.1034
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]


def main():
    rclpy.init()
    node = rclpy.create_node("ik_probe")
    js = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: js.setdefault("m", m), 1)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    cli = node.create_client(GetPositionIK, "/compute_ik")
    cli.wait_for_service(timeout_sec=10)
    seed = JointState()
    for n_, p_ in zip(js["m"].name, js["m"].position):
        if n_ in JOINTS:
            seed.name.append(n_)
            seed.position.append(p_)
    for spec in sys.argv[1:]:
        x, y, z, qx, qy, qz, qw = map(float, spec.split())
        R = _quat_to_R(qx, qy, qz, qw)
        hx, hy, hz = np.array([x, y, z]) - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = hx, hy, hz
        p.orientation.x, p.orientation.y = qx, qy
        p.orientation.z, p.orientation.w = qz, qw
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.timeout.sec = 2
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            print(spec, "-> no answer")
            continue
        if res.error_code.val != 1:
            print(spec, f"-> FAIL {res.error_code.val}")
            continue
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        print(spec, "->", ",".join(f"{sol[j]:.4f}" for j in JOINTS))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
