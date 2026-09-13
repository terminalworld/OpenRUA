import sys, numpy as np, rclpy
sys.argv=[sys.argv[0],""]
from ctl import Ctl, ARM
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
c=Ctl()
def ik(fid, xyz, quat, link=""):
    req=GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    req.ik_request.pose_stamped.header.frame_id=fid
    if link: req.ik_request.ik_link_name=link
    p=req.ik_request.pose_stamped.pose
    p.position.x,p.position.y,p.position.z=xyz
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=quat
    d=c.joints(); js=JointState()
    for j in ARM: js.name.append(j); js.position.append(d[j])
    req.ik_request.robot_state.joint_state=js
    req.ik_request.timeout.sec=1
    f=c.ik.call_async(req); rclpy.spin_until_future_complete(c.node,f,timeout_sec=60)
    r=f.result()
    code=r.error_code.val if r else None
    sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position)) if r and code==1 else {}
    print(repr(fid),link,xyz,quat,"->",code,[round(sol.get(j,0),3) for j in ARM] if sol else "")
ik("", (-0.053,0.0,0.778),(0.924,-0.383,-0.026,0.011))
ik("", (-0.053,0.0,0.778),(1.0,0.0,0.0,0.0))
ik("world", (-0.053,0.0,0.778),(0.924,-0.383,-0.026,0.011))
ik("panda_link0", (0.457,0.0,0.358),(0.924,-0.383,-0.026,0.011))
ik("", (-0.053,0.0,0.778),(1.0,0.0,-0.028,0.0),"panda_hand")
ik("", (-0.21,-0.126,0.723),(1.0,0.0,0.0,0.0),"panda_hand")
ik("", (-0.21,-0.126,0.723),(0.924,-0.383,0,0))
