import rclpy, numpy as np, sys
sys.argv=["x"]
import ctl
c=ctl.Ctl()
from moveit_msgs.srv import GetPositionIK, GetPositionFK
# FK current
req=GetPositionFK.Request(); req.fk_link_names=["panda_hand"]
seed,js=c.arm_seed(); req.robot_state.joint_state=seed
res=c.call(c.fk,req)
print("fk frame", res.pose_stamped[0].header.frame_id)
for ps in res.pose_stamped: 
    p=ps.pose; print(ps.header.frame_id, p.position, p.orientation)
pose=res.pose_stamped[0].pose
def ik(pose, tip=None, frame=""):
    r=GetPositionIK.Request(); r.ik_request.group_name="panda_arm"
    r.ik_request.pose_stamped.header.frame_id=frame
    r.ik_request.pose_stamped.pose=pose
    if tip: r.ik_request.ik_link_name=tip
    r.ik_request.robot_state.joint_state=seed
    r.ik_request.timeout.sec=2
    o=c.call(c.ik,r); print("ik code",o.error_code.val, [round(v,3) for v in o.solution.joint_state.position[:7]] if o.error_code.val==1 else "")
ik(pose)
ik(pose, tip="panda_hand")
ik(pose, tip="panda_link8")
import copy
p2=copy.deepcopy(pose); p2.position.z-=0.2; ik(p2)
p3=copy.deepcopy(pose); p3.position.x+=0.3; p3.position.z-=0.4; ik(p3)
print("---- panda_hand tip tests")
from geometry_msgs.msg import Pose
def mk(x,y,z,q=(1,0,0,0)):
    p=Pose(); p.position.x,p.position.y,p.position.z=float(x),float(y),float(z)
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=[float(v) for v in q]; return p
for tgt in [(0.3,0,0.5),(0.3,0,0.4),(0.3,0,0.3),(0.28,-0.155,0.283),(0.4,-0.2,0.3),(0.5,-0.28,0.25)]:
    print(tgt, end=" "); ik(mk(*tgt), tip="panda_hand")
seed.position=[0.0,-0.785,0.0,-2.356,0.0,1.571,0.785]
print("ready seed")
for tgt in [(0.3,0,0.5),(0.3,0,0.3),(0.28,-0.155,0.283),(0.5,-0.28,0.25)]:
    print(tgt, end=" "); ik(mk(*tgt), tip="panda_hand")
