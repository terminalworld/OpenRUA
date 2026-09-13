import numpy as np, sys
from rob import *
from goto import ik_checked
r = Robot("lk")
def fk_links(q):
    req = GetPositionFK.Request(); req.header.frame_id=""
    req.fk_link_names = [f"panda_link{i}" for i in range(1,9)]+["panda_hand","panda_leftfinger","panda_rightfinger"]
    req.robot_state.joint_state.name=list(JOINTS); req.robot_state.joint_state.position=[float(v) for v in q]
    fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
    res=fut.result()
    for n,p in zip(req.fk_link_names,res.pose_stamped):
        print(f"  {n:18s} {p.pose.position.x:7.3f} {p.pose.position.y:7.3f} {p.pose.position.z:7.3f}")
q0=r.arm_q(); print("current"); fk_links(q0)
R1=np.load("snaps/R_side.npy")
q=ik_checked(r,[-0.201,0.012,1.04],R1,q0); print("target"); fk_links(q)
