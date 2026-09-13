#!/usr/bin/env python3
"""IK probe (no motion). Usage: iktest.py x y z qx qy qz qw"""
import sys
import numpy as np, rclpy, yaml
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = M["actuators"][0]["joints"]
x, y, z, qx, qy, qz, qw = map(float, sys.argv[1:8])
rclpy.init()
node = rclpy.create_node("iktest")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js:
    rclpy.spin_once(node, timeout_sec=0.2)
cur = dict(zip(js["m"].name, js["m"].position))
cli = node.create_client(GetPositionIK, "/compute_ik")
cli.wait_for_service(10)
req = GetPositionIK.Request()
req.ik_request.group_name = "panda_arm"
req.ik_request.ik_link_name = "panda_hand"
req.ik_request.pose_stamped.header.frame_id = ""
p = req.ik_request.pose_stamped.pose
p.position.x, p.position.y, p.position.z = x, y, z
p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
req.ik_request.robot_state.joint_state.name = ARM
req.ik_request.robot_state.joint_state.position = [cur[j] for j in ARM]
req.ik_request.timeout.sec = 2
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
r = fut.result()
print("err", r.error_code.val)
if r.error_code.val == 1:
    sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    print("sol:", [round(sol[j], 4) for j in ARM])
    print("cur:", [round(cur[j], 4) for j in ARM])
rclpy.shutdown()
