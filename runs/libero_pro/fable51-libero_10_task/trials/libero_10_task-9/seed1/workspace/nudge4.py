import numpy as np
from rob import Robot
r = Robot("nudge4")
qn = np.load("snaps/quat_push.npy")
def go(pts, per=3.0, iters=2):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    r.move_q_corrected(qs[-1], seconds=2.0, iters=iters)
    p, qq = r.tcp(); print("TCP", p.round(4), "fingers", np.round(r.fingers(), 4))
print("-- close"); r.gripper(0.0)
X = 0.0; Z = 1.048
print("-- approach"); go([(X, 0.20, Z)], per=3.0)
print("-- push"); go([(X, 0.25, Z), (X, 0.28, Z), (X, 0.30, Z)], per=2.0)
print("-- back off"); go([(X, 0.16, 1.03)], per=3.0, iters=1)
