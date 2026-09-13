import rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node('fk')
js={}
n.create_subscription(JointState,'/joint_states',lambda m: js.setdefault('m',m),1)
while 'm' not in js: rclpy.spin_once(n,timeout_sec=0.2)
cli=n.create_client(GetPositionFK,'/compute_fk'); cli.wait_for_service()
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand']
arm=[f'panda_joint{i}' for i in range(1,8)]
for nm,p in zip(js['m'].name,js['m'].position):
    if nm in arm: req.robot_state.joint_state.name.append(nm); req.robot_state.joint_state.position.append(p)
req.header.frame_id=''
f=cli.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=30)
r=f.result(); print(r.error_code, r.pose_stamped[0].header.frame_id, r.pose_stamped[0].pose)
