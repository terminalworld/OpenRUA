from rob import *
r = Robot()
q = r.arm_q()
req = GetPositionFK.Request(); req.fk_link_names=["panda_link0","panda_hand"]
req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=q
res = r._call(r.fk, req)
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose.position)
# IK at the raw returned pose (no offset)
p=res.pose_stamped[1].pose
xyz=np.array([p.position.x,p.position.y,p.position.z]); R=Rot.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]).as_matrix()
sol=r.solve_ik(xyz+BASE_IN_WORLD, R, seed=q)   # solve_ik subtracts BASE -> raw
print("IK raw-frame:", None if sol is None else np.round(sol,3))
print("current     :", np.round(q,3))
