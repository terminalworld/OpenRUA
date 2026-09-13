import rclpy, yaml, sys
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); node=rclpy.create_node("fk")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
arm=[f"panda_joint{i}" for i in range(1,8)]
cur=dict(zip(js["m"].name,js["m"].position))
print("joints:", [round(cur[j],4) for j in arm], "fingers:", [round(cur[f"panda_finger_joint{i}"],4) for i in (1,2)])
cli=node.create_client(GetPositionFK,"/compute_fk"); cli.wait_for_service()
req=GetPositionFK.Request(); req.fk_link_names=["panda_hand"]
req.robot_state.joint_state.name=arm; req.robot_state.joint_state.position=[cur[j] for j in arm]
fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut,timeout_sec=30)
r=fut.result()
p=r.pose_stamped[0].pose
print("frame:", r.pose_stamped[0].header.frame_id, "err", r.error_code.val)
print(f"hand (base frame): {p.position.x:.4f} {p.position.y:.4f} {p.position.z:.4f}  q {p.orientation.x:.4f} {p.orientation.y:.4f} {p.orientation.z:.4f} {p.orientation.w:.4f}")
print(f"hand (world): {p.position.x-0.51:.4f} {p.position.y:.4f} {p.position.z+0.42:.4f}")
rclpy.shutdown()
