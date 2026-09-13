from rob import *
r = Robot()
p, q, tcp = r.hand_pose()
log("hand", p.round(3), "quat", np.round(q,3), "tcp", tcp.round(3))
log("fingers", r.fingers())
for name,(x,y,z) in {"soup_pre":(-0.244,-0.173,0.60),"soup_grasp":(-0.244,-0.173,0.45),
                     "tom_pre":(-0.023,-0.274,0.62),"tom_grasp":(-0.023,-0.274,0.46),
                     "basket_a":(0.0,0.215,0.72),"basket_b":(0.0,0.285,0.72)}.items():
    for yaw in (0.0, 0.785, -0.785, 1.571):
        sol = r.solve_ik(x,y,z,topdown_quat(yaw))
        log(name, "yaw",yaw, None if sol is None else np.round(sol,3))
        if sol is not None: break
