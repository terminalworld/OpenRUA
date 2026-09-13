import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
import ctl
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
c=ctl.Ctl()
def try_ik(pos, quat, link="", frame="", avoid=True, seed=True):
    req=GetPositionIK.Request()
    req.ik_request.group_name="panda_arm"
    req.ik_request.ik_link_name=link
    req.ik_request.pose_stamped.header.frame_id=frame
    p=req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z=pos
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=quat
    req.ik_request.avoid_collisions=avoid
    if seed:
        s=JointState(); s.name=list(ctl.ARM); s.position=[float(v) for v in c.arm_q()]
        req.ik_request.robot_state.joint_state=s
    req.ik_request.timeout.sec=2
    f=c.ik.call_async(req); rclpy.spin_until_future_complete(c.n,f,timeout_sec=60)
    r=f.result()
    if r is None: print(pos,quat,link,frame,"-> NO ANSWER"); return
    print(pos,quat,link,frame,avoid,"-> code",r.error_code.val, np.round(r.solution.joint_state.position,3).tolist()[:9] if r.error_code.val==1 else "")
hb=(0.457,0.0,0.3576); q=(0.9996,0.0,-0.0284,0.0)
print("---- world coords with empty frame")
try_ik((-0.053,0.0,0.7776),q)
try_ik((-0.053,0.0,0.7776),q,frame="world")
try_ik((-0.053,0.0,0.7776),q,link="panda_hand")
try_ik((-0.053,0.0,0.7776),q,link="panda_link8")
