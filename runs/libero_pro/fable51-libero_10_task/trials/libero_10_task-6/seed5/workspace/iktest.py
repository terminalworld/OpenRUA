#!/usr/bin/env python3
"""Probe which frame /compute_ik interprets poses in."""
import sys
import rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("iktest")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js:
    rclpy.spin_once(node, timeout_sec=0.2)
arm = [f"panda_joint{i}" for i in range(1, 8)]
cur = dict(zip(js["m"].name, js["m"].position))
seed = JointState()
for n in arm:
    seed.name.append(n); seed.position.append(cur[n])
cli = node.create_client(GetPositionIK, "/compute_ik")
cli.wait_for_service(timeout_sec=10)

def ik(x, y, z, q=(0.9996, 0.0, -0.0284, 0.0)):
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    req.ik_request.robot_state.joint_state = seed
    req.ik_request.ik_link_name = "panda_hand"
    req.ik_request.timeout.sec = 2
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    r = fut.result()
    if r is None:
        return "timeout"
    if r.error_code.val != 1:
        return f"err {r.error_code.val}"
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    return [round(sol[j], 3) for j in arm]

print("current:", [round(cur[j], 3) for j in arm])
print("world coords (-0.053,0,0.7776):", ik(-0.053, 0.0, 0.7776))
print("base  coords (0.457,0,0.3576):", ik(0.457, 0.0, 0.3576))
rclpy.shutdown()
