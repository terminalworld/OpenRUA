import rclpy, sys
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node('iktest')
js={}
n.create_subscription(JointState,'/joint_states',lambda m: js.setdefault('m',m),1)
while 'm' not in js: rclpy.spin_once(n,timeout_sec=0.2)
cli=n.create_client(GetPositionIK,'/compute_ik'); cli.wait_for_service(10)
def ik(x,y,z,qx,qy,qz,qw,frame=''):
    req=GetPositionIK.Request(); r=req.ik_request
    r.group_name='panda_arm'; r.pose_stamped.header.frame_id=frame
    p=r.pose_stamped.pose; p.position.x,p.position.y,p.position.z=float(x),float(y),float(z)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=float(qx),float(qy),float(qz),float(qw)
    seed=JointState()
    for a,b in zip(js['m'].name,js['m'].position):
        if a.startswith('panda_joint'): seed.name.append(a); seed.position.append(b)
    r.robot_state.joint_state=seed
    f=cli.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=60)
    res=f.result()
    if res is None: print('timeout'); return
    print('code',res.error_code.val, [f"{v:.3f}" for k,v in zip(res.solution.joint_state.name,res.solution.joint_state.position) if k.startswith('panda_joint')])
print('current', [f"{v:.3f}" for k,v in zip(js['m'].name,js['m'].position) if k.startswith('panda_joint')])
print('base-frame coords:'); ik(0.4570,0.0,0.3576,0.9996,0,-0.0284,0)
print('world-frame coords:'); ik(-0.0530,0.0,0.7776,0.9996,0,-0.0284,0)
