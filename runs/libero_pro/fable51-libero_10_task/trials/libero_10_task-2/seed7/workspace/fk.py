import rclpy, sys
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node('fk')
cli=n.create_client(GetPositionFK,'/compute_fk'); cli.wait_for_service()
req=GetPositionFK.Request(); req.fk_link_names=['panda_hand','panda_link8']
req.header.frame_id=''
js=JointState(); js.name=[f'panda_joint{i}' for i in range(1,8)]
js.position=[float(x) for x in sys.argv[1].split(',')] if len(sys.argv)>1 else [0.0,-0.1610,0.0,-2.4446,0.0,2.2268,0.7854]
req.robot_state.joint_state=js
f=cli.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=30)
r=f.result(); print(r.error_code)
for nm,ps in zip(r.fk_link_names,r.pose_stamped):
    p=ps.pose.position;q=ps.pose.orientation; print(nm,ps.header.frame_id,f'{p.x:.4f} {p.y:.4f} {p.z:.4f} q {q.x:.4f} {q.y:.4f} {q.z:.4f} {q.w:.4f}')
