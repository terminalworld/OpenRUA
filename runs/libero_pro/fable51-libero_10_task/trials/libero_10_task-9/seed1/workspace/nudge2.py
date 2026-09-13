import numpy as np
from rob import Robot
r = Robot("nudge2")
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
X = -0.016; Z = 1.00
print("-- push"); go([(X, 0.27, Z), (X, 0.29, Z)], per=2.5)
print("-- back off"); go([(X, 0.22, 1.02), (X, 0.12, 1.15)], per=3.0, iters=1)
