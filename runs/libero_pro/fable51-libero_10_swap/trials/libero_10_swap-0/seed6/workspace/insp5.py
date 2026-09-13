from rob import *
from scipy.spatial.transform import Rotation as Rot
from moveit_msgs.srv import GetPositionFK
r=Robot('insp5')
def links(q, names=('panda_link5','panda_link7','panda_hand')):
    req=GetPositionFK.Request(); req.fk_link_names=list(names)
    req.robot_state.joint_state=r._seed(q)
    fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
    return {n:np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],3) for n,ps in zip(names,fut.result().pose_stamped)}
def q_roll(alpha):  # wrist displaced toward -y, fingers tilted in y-z plane
    R=Rot.from_euler('x',alpha).as_matrix()@np.diag([1,-1,-1])
    return tuple(Rot.from_matrix(R).as_quat())
def q_pitch(theta):  # wrist toward +x (theta>0) or -x (theta<0), fingers along y
    c,s=np.cos(theta),np.sin(theta)
    R=np.array([[c,0,-s],[0,-1,0],[-s,0,-c]])
    return tuple(Rot.from_matrix(R).as_quat())
x,y=-0.175,-0.154
q=r.joints()
for label,quat in [('vertical',Q_DOWN),('roll25',q_roll(np.radians(25))),('roll35',q_roll(np.radians(35))),('pitch-25',q_pitch(np.radians(-25))),('pitch20',q_pitch(np.radians(20)))]:
    print('---',label)
    seed=q
    for z in [0.65,0.60,0.55,0.50,0.45]:
        s=r.ik_tcp_world((x,y,z), quat=quat, seed=seed)
        if s is None: print(z,None); continue
        print(z, np.round(s,3), links(s) if z in (0.45,) else '')
        seed=s
