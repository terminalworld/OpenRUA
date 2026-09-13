from ctl import *
c = Ctl()
print("arm q", np.round(c.arm_q(),4)); print("fingers", c.fingers())
p,q = c.hand_world(); print("hand world", p.round(4), np.round(q,4))
t,_ = c.tcp_world(); print("tcp world", t.round(4))
# FK check in planner frame
req = GetPositionFK.Request(); req.fk_link_names=["panda_hand"]; s=JointState(); s.name=list(JOINTS); s.position=c.arm_q(); req.robot_state.joint_state=s
f=c.fk.call_async(req); rclpy.spin_until_future_complete(c.n,f,timeout_sec=30); r=f.result()
pp=r.pose_stamped[0].pose.position; print("FK panda_hand frame", r.pose_stamped[0].header.frame_id, round(pp.x,4),round(pp.y,4),round(pp.z,4), "code", r.error_code.val)
# IK sanity: solve for current tcp, expect near current joints
q,code=c.solve_ik(t, q, at_tcp=True); print("IK back to current:", code, None if q is None else np.round(np.array(q)-np.array(c.arm_q()),3))
print("yaw_down(0)", yaw_down(0), "R col2", quat_R(*yaw_down(0))[:,2], "R col1", quat_R(*yaw_down(0))[:,1])
print("yaw_down(pi/2) R col1", quat_R(*yaw_down(np.pi/2))[:,1].round(3), "col2", quat_R(*yaw_down(np.pi/2))[:,2].round(3))
