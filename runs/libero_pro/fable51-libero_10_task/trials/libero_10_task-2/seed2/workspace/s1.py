from rob import *
r = R()
req = GetPositionFK.Request()
req.header.frame_id = ""
req.fk_link_names = ["panda_hand", "panda_link0", "panda_link8"]
req.robot_state.joint_state = r._seed()
print(req.robot_state.joint_state)
fut = r.fkc.call_async(req)
rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
res = fut.result()
print(res.error_code, res.fk_link_names)
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose.position)
# IK without timeout, like ik_move.py
req = GetPositionIK.Request()
req.ik_request.group_name = "panda_arm"
req.ik_request.pose_stamped.header.frame_id = ""
p = req.ik_request.pose_stamped.pose
p.position.x, p.position.y, p.position.z = 0.4, 0.0, 0.4
p.orientation.x = 1.0
req.ik_request.robot_state.joint_state = r._seed()
fut = r.ikc.call_async(req)
rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
res = fut.result(); print("ik", res.error_code, res.solution.joint_state.name, np.round(res.solution.joint_state.position,3))
