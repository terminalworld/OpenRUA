import numpy as np, sys
from ctl import *; from mp import *
r=Robot("s3"); pl=Planner(r)
pl.scene([],remove=["mug","mug_body","mug_rim","mug_handle"]); print("scene",pl.scene(scene_objects()))
go="--go" in sys.argv
A=[0.29,0.33,-0.89,-2.67,-2.34,3.5,2.59]
tr=None
for attempt in range(6):
    tr=pl.plan_joints(A,t=60.0,attempts=4)
    if tr is not None: break
if tr is None: sys.exit(1)
P=np.array([p.positions for p in tr.points]); print("joint ranges min",P.min(0).round(2)," max",P.max(0).round(2))
# FK sweep of hand + fingertip
from moveit_msgs.srv import GetPositionFK
def fk(q):
    req=GetPositionFK.Request(); req.header.frame_id=""; req.fk_link_names=["panda_hand"]
    req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=list(map(float,q))
    f=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30); p=f.result().pose_stamped[0].pose
    R=quat_R(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w); o=np.array([p.position.x,p.position.y,p.position.z])
    return o, o+0.1034*R[:,2]
minz=9; 
for i in range(0,len(P),max(1,len(P)//20)):
    o,t=fk(P[i]); minz=min(minz,t[2]); print(i,"hand",o.round(3),"tip",t.round(3))
o,t=fk(P[-1]); print("final hand",o.round(4),"tip",t.round(4))
if go:
    pl.execute(tr); p,q=r.hand_pose(); print("hand",p.round(4),q.round(3),"fingers",r.finger())
    r.snap("robot0_eye_in_hand","/workspace/s3_eih.png"); r.snap("sideview","/workspace/s3_side.png"); r.snap("birdview","/workspace/s3_bird.png")
