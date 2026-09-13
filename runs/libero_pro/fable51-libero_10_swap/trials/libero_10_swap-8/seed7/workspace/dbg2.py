from rob import *
r = Robot()
for yaw in (0, -math.pi/2):
    q = r.ik_tcp((0.12, 0.0, 1.2), grasp_R(yaw,0))
    pos, quat, tcp = r.fk_hand(q)
    print("yaw", round(yaw,2), "q", np.round(q,3), "tcp", np.round(tcp,4), "hand R=\n", np.round(R_from_q(*quat),3))
