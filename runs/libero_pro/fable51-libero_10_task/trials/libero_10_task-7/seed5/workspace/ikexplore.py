import sys, numpy as np, rclpy
sys.argv=[sys.argv[0],""]
from ctl import Ctl, ARM, TCP, quat_R
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as Rot
c=Ctl()
def ik(xyz, quat, seed):
    req=GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    req.ik_request.ik_link_name="panda_hand"
    p=req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z=map(float,xyz)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,quat)
    js=JointState(); js.name=list(ARM); js.position=[float(v) for v in seed]
    req.ik_request.robot_state.joint_state=js
    req.ik_request.timeout.sec=1
    f=c.ik.call_async(req); rclpy.spin_until_future_complete(c.node,f,timeout_sec=60)
    r=f.result()
    if not r or r.error_code.val!=1: return None
    sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))
    return np.array([sol[j] for j in ARM])
tcp=np.array([-0.21,-0.126,0.47])
cur=np.array(c.arm_q())
seeds={"cur":cur,"ready":[0,-0.785,0,-2.356,0,1.571,0.785],"ready_j5pi":[0,-0.785,0,-2.356,np.pi/2,1.571,0.785],
       "ready_j5mpi":[0,-0.785,0,-2.356,-np.pi/2,1.571,0.785],"j1flip":[np.pi/2,-0.785,-np.pi/2,-2.356,0,1.571,0.785],
       "home":[0,-0.161,0,-2.44,0,2.23,0.785],"alt":[0.5,0.3,-0.8,-2.5,0.5,2.6,0.0],"alt2":[-0.5,0.3,0.8,-2.5,-0.5,2.6,1.5]}
for pitch_deg in [0,15,25,35]:
  for yaw_deg in [0,45,-45,90]:
    # rotation: start from down (1,0,0,0) i.e. R=diag(1,-1,-1); pitch about world y (tilt fingers toward -x when negative?)
    Rd=quat_R(1,0,0,0)
    R=Rot.from_euler('z',yaw_deg,degrees=True).as_matrix()@Rot.from_euler('y',pitch_deg,degrees=True).as_matrix()@Rd
    q=Rot.from_matrix(R).as_quat()
    hand=tcp-TCP*R[:,2]
    for name,s in seeds.items():
        sol=ik(hand,q,s)
        if sol is not None:
            print(f"pitch{pitch_deg} yaw{yaw_deg} seed {name}: ", sol.round(2), "zaxis",R[:,2].round(2))
