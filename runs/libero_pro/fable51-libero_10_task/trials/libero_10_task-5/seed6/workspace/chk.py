import numpy as np, json, rclpy, sys
from ctl import *; from valid import check
from moveit_msgs.srv import GetStateValidity, GetPositionFK
r=Robot("chk"); chain=json.load(open(sys.argv[1] if len(sys.argv)>1 else "chain.json"))
c=r.node.create_client(GetStateValidity,'/check_state_validity')
def fk(q):
    req=GetPositionFK.Request(); req.header.frame_id=""; req.fk_link_names=["panda_hand"]
    req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=list(map(float,q))
    f=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30); p=f.result().pose_stamped[0].pose
    R=quat_R(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w); o=np.array([p.position.x,p.position.y,p.position.z])
    return o, o+0.1034*R[:,2]
for (z0,d0,q0),(z1,d1,q1) in zip(chain[:-1],chain[1:]):
    q0=np.array(q0); q1=np.array(q1); bad=[]; tips=[]
    for a in np.linspace(0,1,7):
        q=q0+(q1-q0)*a; v,cs=check(r,q,c); o,t=fk(q); tips.append(t)
        if not v: bad.append((round(a,2),cs))
    tips=np.array(tips); print("seg %.3f->%.3f tip y range %.3f..%.3f x %.3f..%.3f z %.3f..%.3f bad=%s"%(z0,z1,tips[:,1].min(),tips[:,1].max(),tips[:,0].min(),tips[:,0].max(),tips[:,2].min(),tips[:,2].max(),bad))
