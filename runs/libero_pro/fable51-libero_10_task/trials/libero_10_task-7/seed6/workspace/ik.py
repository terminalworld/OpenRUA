#!/usr/bin/env python3
"""IK query only (no motion). Usage: ik.py x y z qx qy qz qw  -> prints joints."""
import sys, time
import rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

ARM = [f"panda_joint{i}" for i in range(1, 8)]


def solve(node, cli, seed, pose, timeout=60):
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.ik_link_name = "panda_hand"
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = pose[:3]
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = pose[3:]
    req.ik_request.robot_state.joint_state.name = ARM
    req.ik_request.robot_state.joint_state.position = list(seed)
    req.ik_request.avoid_collisions = False
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    r = fut.result()
    if r is None or r.error_code.val != 1:
        return None, (r.error_code.val if r else None)
    d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    return [d[j] for j in ARM], 1


if __name__ == "__main__":
    pose = [float(a) for a in sys.argv[1:8]]
    rclpy.init()
    node = rclpy.create_node("ikq")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    end = time.time() + 15
    while "m" not in js and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    d = dict(zip(js["m"].name, js["m"].position))
    seed = [d[j] for j in ARM]
    cli = node.create_client(GetPositionIK, "/compute_ik")
    cli.wait_for_service(10)
    sol, code = solve(node, cli, seed, pose)
    print("seed:", ", ".join(f"{v:.4f}" for v in seed))
    print("code:", code, "sol:", None if sol is None else ", ".join(f"{v:.4f}" for v in sol))
    rclpy.shutdown()
