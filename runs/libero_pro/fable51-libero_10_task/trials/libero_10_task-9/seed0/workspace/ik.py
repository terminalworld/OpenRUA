#!/usr/bin/env python3
"""IK helper: python3 ik.py x y z qx qy qz qw  -> prints joint solution (world-frame pose of panda_hand).
Optional: --seed j1,...,j7 ; --link tcp (pose given for TCP, converted to hand)
"""
import sys
import numpy as np
import rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def solve(node, cli, pose, seed=None, timeout=60.0):
    """pose = (x,y,z,qx,qy,qz,qw) of panda_hand in world. Returns list of 7 or None."""
    if seed is None:
        js = {}
        sub = node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
        while "m" not in js:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_subscription(sub)
        d = dict(zip(js["m"].name, js["m"].position))
        seed = [d[j] for j in JOINTS]
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = map(float, pose[:3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, pose[3:])
    req.ik_request.robot_state.joint_state.name = JOINTS
    req.ik_request.robot_state.joint_state.position = [float(s) for s in seed]
    req.ik_request.avoid_collisions = False
    req.ik_request.timeout.sec = 2
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    res = fut.result()
    if res is None or res.error_code.val != 1:
        return None
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    return [sol[j] for j in JOINTS]


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    x, y, z, qx, qy, qz, qw = map(float, args[:7])
    if "--link" in sys.argv and "tcp" in sys.argv:
        R = quat_R(qx, qy, qz, qw)
        x, y, z = np.array([x, y, z]) - TCP * R[:, 2]
    seed = None
    if "--seed" in sys.argv:
        seed = [float(v) for v in sys.argv[sys.argv.index("--seed") + 1].split(",")]
    rclpy.init()
    node = rclpy.create_node("ik_helper")
    cli = node.create_client(GetPositionIK, "/compute_ik")
    cli.wait_for_service(10)
    sol = solve(node, cli, (x, y, z, qx, qy, qz, qw), seed)
    if sol is None:
        print("IK FAILED"); sys.exit(1)
    print(",".join(f"{v:.5f}" for v in sol))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
