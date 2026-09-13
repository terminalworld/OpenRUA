import numpy as np, rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
rclpy.init(); node = rclpy.create_node("fkt")
got={}
sub=node.create_subscription(JointState,"/joint_states",lambda m: got.setdefault("m",m),1)
while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
js=dict(zip(got["m"].name,got["m"].position))
cli=node.create_client(GetPositionFK,"/compute_fk"); cli.wait_for_service(10)
req=GetPositionFK.Request(); req.fk_link_names=["panda_link8","panda_hand","panda_hand_tcp","panda_link0"]
req.robot_state.joint_state.name=JOINTS; req.robot_state.joint_state.position=[js[j] for j in JOINTS]
f=cli.call_async(req); rclpy.spin_until_future_complete(node,f,timeout_sec=60)
r=f.result(); print("code",r.error_code.val)
for n,p in zip(r.fk_link_names,r.pose_stamped):
    print(n,p.header.frame_id,round(p.pose.position.x,4),round(p.pose.position.y,4),round(p.pose.position.z,4),
          [round(v,4) for v in (p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w)])
rclpy.shutdown()
