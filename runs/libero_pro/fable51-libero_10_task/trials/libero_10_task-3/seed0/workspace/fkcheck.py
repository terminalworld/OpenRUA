import rclpy, numpy as np
from rob import Robot, ARM
from moveit_msgs.srv import GetPositionFK
r=Robot("fkc")
q=r.arm_q()
req=GetPositionFK.Request(); req.header.frame_id=""
req.fk_link_names=["panda_link0","panda_link1","panda_hand","panda_leftfinger"]
req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=list(map(float,q))
fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
res=fut.result()
for n,ps in zip(res.fk_link_names,res.pose_stamped):
    p=ps.pose.position; o=ps.pose.orientation
    print(n, "frame=",repr(ps.header.frame_id), "pos=",round(p.x,4),round(p.y,4),round(p.z,4), "q=",round(o.x,4),round(o.y,4),round(o.z,4),round(o.w,4))
