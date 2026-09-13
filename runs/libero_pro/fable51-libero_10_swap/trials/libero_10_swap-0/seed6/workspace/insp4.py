from rob import *
from scipy.spatial.transform import Rotation as Rot
from moveit_msgs.srv import GetPositionFK
r=Robot('insp4')
def q_tilt(theta, yaw=0.0):
    c,s=np.cos(theta),np.sin(theta)
    R=np.array([[c,0,-s],[0,-1,0],[-s,0,-c]])
    R=Rot.from_euler('z',yaw).as_matrix()@R
    return tuple(Rot.from_matrix(R).as_quat())
def links(q, names=('panda_link5','panda_link7','panda_hand')):
    req=GetPositionFK.Request(); req.fk_link_names=list(names)
    req.robot_state.joint_state=r._seed(q)
    fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
    return {n:np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],3) for n,ps in zip(names,fut.result().pose_stamped)}
x,y=-0.25,-0.149
q=r.joints()
for deg,yawdeg in [(35,-45),(40,-45),(35,-60),(40,-30)]:
    quat=q_tilt(np.radians(deg),np.radians(yawdeg)); print('--- tilt',deg,'yaw',yawdeg)
    seed=q
    for z in [0.66,0.61,0.56,0.51,0.461]:
        s=r.ik_tcp_world((x,y,z), quat=quat, seed=seed)
        if s is None: print(z,None); continue
        print(z, np.round(s,3), links(s) if z in (0.461,0.61) else '')
        seed=s
