import rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node('fktest')
js={}
n.create_subscription(JointState,'/joint_states',lambda m: js.setdefault('m',m),1)
while 'm' not in js: rclpy.spin_once(n,timeout_sec=0.2)
cli=n.create_client(GetPositionFK,'/compute_fk'); print('svc',cli.wait_for_service(10))
req=GetPositionFK.Request()
req.header.frame_id=''
req.fk_link_names=['panda_link8','panda_hand','panda_link0']
seed=JointState()
for a,b in zip(js['m'].name,js['m'].position):
    if a.startswith('panda_joint'): seed.name.append(a); seed.position.append(b)
req.robot_state.joint_state=seed
f=cli.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=60)
res=f.result()
print('code',res.error_code.val)
for name,ps in zip(res.fk_link_names,res.pose_stamped):
    p=ps.pose.position;q=ps.pose.orientation
    print(name, ps.header.frame_id, f"({p.x:.4f},{p.y:.4f},{p.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
