import numpy as np, sys
from rob import *
r = Robot("stage3")
C = np.array([-0.1967, -0.1995])
Q_G = (0.7071, -0.7071, 0, 0)
def go(tcp, seconds=3.0, seed=None, via_n=0):
    seed = r.arm_q() if seed is None else seed
    q = r.ik_tcp_world(tcp, Q_G, seed=seed, timeout=5)
    if q is None: raise SystemExit(f"IK fail {tcp}")
    via = []
    if via_n:
        cur, _ = r.tcp_world()
        s = seed
        for i in range(1, via_n+1):
            p = cur + (np.asarray(tcp)-cur)*i/(via_n+1)
            s = r.ik_tcp_world(p, Q_G, seed=s, timeout=5)
            if s is None: raise SystemExit(f"IK fail via {p}")
            via.append(s)
    code, err = r.move_q(q, seconds, via=via or None)
    if err > 0.01:
        print("  resending"); code, err = r.move_q(q, seconds)
    tcp_now, fq = r.tcp_world(); print("  tcp", np.round(tcp_now,4), "quat", np.round(fq,3), flush=True)
    return q
print("fingers", r.fingers())
go((C[0], C[1], 1.15), 3.0)
go((C[0], C[1], 1.048), 3.0, via_n=2)
r.snap("robot0_eye_in_hand", "/workspace/eih_pregrasp.png")
f = r.gripper(0.0)
r.snap("agentview", "/workspace/agentview_grasp.png")
go((C[0], C[1], 1.25), 3.0, via_n=1)
print("fingers after lift", r.fingers())
r.snap("agentview", "/workspace/agentview_lift.png"); r.snap("sideview", "/workspace/sideview_lift.png")
