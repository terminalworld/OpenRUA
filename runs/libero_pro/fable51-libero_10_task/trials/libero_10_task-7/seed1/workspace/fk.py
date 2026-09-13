import rclpy, yaml
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); node=rclpy.create_node("fk")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
cli=node.create_client(GetPositionFK,"/compute_fk"); cli.wait_for_service()
req=GetPositionFK.Request(); req.fk_link_names=["panda_hand"]
arm=[f"panda_joint{i}" for i in range(1,8)]
d=dict(zip(js["m"].name,js["m"].position))
req.robot_state.joint_state.name=arm; req.robot_state.joint_state.position=[d[a] for a in arm]
f=cli.call_async(req); rclpy.spin_until_future_complete(node,f,timeout_sec=30)
r=f.result()
print("joints:",[round(d[a],4) for a in arm], "fingers:", [round(d[n],4) for n in d if "finger" in n])
for ps in r.pose_stamped:
    p=ps.pose.position; q=ps.pose.orientation
    print(f"frame={ps.header.frame_id} pos=({p.x:.4f},{p.y:.4f},{p.z:.4f}) quat=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
