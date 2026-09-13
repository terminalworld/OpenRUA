from rob import *
r = Robot()
cur = r.arm_positions()
q = topdown_quat(0)
sol = r.solve_ik([-0.208, -0.163, 0.62], q, cur)
print("sol", np.round(sol, 4).tolist())
# FK of the solution
req = GetPositionFK.Request(); req.header.frame_id = ""; req.fk_link_names = ["panda_hand", "panda_link8"]
js = JointState()
for n, v in zip(ARM, sol): js.name.append(n); js.position.append(float(v))
req.robot_state.joint_state = js
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
for ps in fut.result().pose_stamped:
    p = ps.pose; print(ps.header.frame_id, [round(v,4) for v in (p.position.x,p.position.y,p.position.z)], [round(v,4) for v in (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)])
r.close()
