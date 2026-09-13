import numpy as np, sys
from rob import *
r = Robot("stage6")
C = np.array([-0.0351, 0.2385])
Q_G = (0.7071, -0.7071, 0, 0)
def go(tcp, quat, seconds=3.0, via=None):
    seed = r.arm_q()
    q = r.ik_tcp_world(tcp, quat, seed=seed, timeout=5)
    if q is None: raise SystemExit(f"IK fail {tcp}")
    vias = []
    if via:
        s = seed
        for p in via:
            s = r.ik_tcp_world(p, quat, seed=s, timeout=5)
            if s is None: raise SystemExit(f"IK fail via {p}")
            vias.append(s)
    code, err = r.move_q(q, seconds, via=vias or None)
    tries = 0
    while err > 0.01 and tries < 3:
        print("  resending"); code, err = r.move_q(q, seconds); tries += 1
    tcp_now, fq = r.tcp_world(); R = quat_to_R(*fq)
    print("  tcp", np.round(tcp_now,4), "hz", np.round(R[:,2],3), "finger axis", np.round(R[:,1],3), "fingers", np.round(r.fingers(),4), flush=True)
    return q
print("q now", np.round(r.arm_q(),3))
go((C[0], C[1], 1.15), Q_G, 3.0)
go((C[0], C[1], 1.048), Q_G, 3.0, via=[(C[0], C[1], 1.10)])
r.snap("robot0_eye_in_hand", "/workspace/eih_B_pregrasp.png")
r.gripper(0.0)
go((C[0], C[1], 1.25), Q_G, 3.0, via=[(C[0], C[1], 1.15)])
print("fingers after lift", r.fingers())
r.snap("sideview", "/workspace/sideview_liftB.png")
