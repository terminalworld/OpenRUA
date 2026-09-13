#!/usr/bin/env python3
import sys, numpy as np, rclpy
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
ARM = [f"panda_joint{i}" for i in range(1, 8)]
rclpy.init(); node = rclpy.create_node("iktest")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
while "m" not in js: rclpy.spin_once(node, timeout_sec=0.2)
d = dict(zip(js["m"].name, js["m"].position))
cur = [d[j] for j in ARM]
fk = node.create_client(GetPositionFK, "/compute_fk"); fk.wait_for_service(10)
req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
req.robot_state.joint_state.name = ARM; req.robot_state.joint_state.position = cur
fut = fk.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
ps = fut.result().pose_stamped[0]
print("FK header frame:", repr(ps.header.frame_id), "pos:", ps.pose.position)
ik = node.create_client(GetPositionIK, "/compute_ik"); ik.wait_for_service(10)
for label, off in [("world-coords", (0, 0, 0)), ("base-coords", (0.51, 0, -0.42))]:
    r = GetPositionIK.Request(); r.ik_request.group_name = "panda_arm"
    r.ik_request.pose_stamped.header.frame_id = ""
    r.ik_request.pose_stamped.pose = ps.pose
    r.ik_request.pose_stamped.pose.position.x = ps.pose.position.x + off[0]
    r.ik_request.pose_stamped.pose.position.y = ps.pose.position.y + off[1]
    r.ik_request.pose_stamped.pose.position.z = ps.pose.position.z + off[2]
    r.ik_request.robot_state.joint_state.name = ARM
    r.ik_request.robot_state.joint_state.position = cur
    r.ik_request.timeout.sec = 2
    f = ik.call_async(r); rclpy.spin_until_future_complete(node, f, timeout_sec=60)
    res = f.result()
    if res is None or res.error_code.val != 1:
        print(label, "IK failed", None if res is None else res.error_code.val); continue
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    print(label, "sol:", [round(sol[j], 3) for j in ARM], "cur:", [round(c, 3) for c in cur])
rclpy.shutdown()
