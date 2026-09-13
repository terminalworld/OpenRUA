from rob import *
import rob
r=Rob("t")
q=r.arm_q()
# raw FK
req=GetPositionFK.Request(); req.header.frame_id=""; req.fk_link_names=["panda_hand","panda_link0"]
req.robot_state.joint_state.name=list(JOINTS); req.robot_state.joint_state.position=[float(x) for x in q]
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
for ps in fut.result().pose_stamped: print(ps.header.frame_id, ps.pose.position)
# test IK in world coordinates vs base coordinates
rob.BASE=np.zeros(3)
p,R=r.fk(q)
print("world-frame IK:", None if (s:=r.ik(p,R)) is None else (s-q).round(3))
rob.BASE=np.array([-0.66,0,0.912])
print("base-frame IK:", None if (s:=r.ik(p,R)) is None else (s-q).round(3))
