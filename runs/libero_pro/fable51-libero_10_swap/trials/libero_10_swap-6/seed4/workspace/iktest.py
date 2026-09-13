from arm import *
a = Arm()
def fk_of(q):
    a.fk.wait_for_service(10); req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = list(q)
    res = a.spin(a.fk.call_async(req), 60)
    out=[]
    for ps in res.pose_stamped:
        p=ps.pose; qq=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
        out.append((np.round([p.position.x,p.position.y,p.position.z],4), np.round(qq,4), Rot.from_quat(qq).as_matrix()[:,1].round(3)))
    return out
cur = a.arm_q()
print("current q", np.round(cur,4)); print("  fk hand/link8:", fk_of(cur))
for yaw in [0, 45, -45, 90]:
    pos, q = tcp_to_hand([-0.094,-0.195,0.66], yaw)
    sol = a.solve_ik(pos, q, seed=cur)
    print(f"yaw {yaw}: req quat {np.round(q,4)}  sol {None if sol is None else np.round(sol,3)}")
    if sol: print("   fk:", fk_of(sol))
