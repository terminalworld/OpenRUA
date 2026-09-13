import numpy as np, sys
from rob import *
r = Robot("stage4")
def rot_y(th):
    c,s=np.cos(th),np.sin(th); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
PITCH = np.radians(12)
Q_P = tuple(R_to_quat(rot_y(-PITCH) @ quat_to_R(1,0,0,0)))   # fingers along y, hand z tilted toward +x
HANG = 0.148    # TCP -> pot bottom (knob grasp), pot on table: TCP 1.048, bottom 0.90
PLATE = 0.932
Y_PLACE = float(sys.argv[1]) if len(sys.argv) > 1 else -0.008
X_BOTTOM = 0.185
Z_MARGIN = float(sys.argv[2]) if len(sys.argv) > 2 else 0.008
hz = np.array([np.sin(PITCH), 0, -np.cos(PITCH)])
tcp_place = np.array([X_BOTTOM, Y_PLACE, PLATE + Z_MARGIN + 0.037*np.sin(PITCH)]) - HANG*hz
print("place TCP", np.round(tcp_place,4), "expected bottom center", np.round(tcp_place + HANG*hz,4))

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
    if err > 0.01:
        print("  resending"); code, err = r.move_q(q, seconds)
    tcp_now, fq = r.tcp_world(); R = quat_to_R(*fq)
    print("  tcp", np.round(tcp_now,4), "hz", np.round(R[:,2],3), "fingers", np.round(r.fingers(),4), flush=True)
    return q

cur, _ = r.tcp_world(); print("start tcp", np.round(cur,4))
pre = tcp_place + np.array([0, 0, 0.10])
mid = (cur + pre)/2; mid[2] = 1.28
go(pre, Q_P, 5.0, via=[mid])
r.snap("agentview", "/workspace/agentview_preplace.png")
go(tcp_place, Q_P, 3.0, via=[tcp_place + np.array([0,0,0.05])])
r.snap("sideview", "/workspace/sideview_place.png")
r.gripper(0.04)
go(tcp_place + np.array([0,0,0.12]), Q_P, 3.0)
r.snap("agentview", "/workspace/agentview_placed.png"); r.snap("sideview", "/workspace/sideview_placed.png"); r.snap("birdview", "/workspace/birdview_placed.png")
