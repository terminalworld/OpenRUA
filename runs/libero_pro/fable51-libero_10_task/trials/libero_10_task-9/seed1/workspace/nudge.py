import numpy as np
from rob import Robot, approach_quat
r = Robot("nudge")
phi = np.deg2rad(30)
qn = approach_quat(np.array([0, np.cos(phi), -np.sin(phi)]), np.array([1.0, 0, 0]))
np.save("snaps/quat_push.npy", qn)
def go(pts, per=3.0, iters=2):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    r.move_q_corrected(qs[-1], seconds=2.0, iters=iters)
    p, qq = r.tcp(); print("TCP", p.round(4), "quat", qq.round(3), "fingers", np.round(r.fingers(), 4))
X = -0.016; Z = 1.00
print("-- approach"); go([(X, 0.12, 1.15), (X, 0.20, 1.02), (X, 0.245, Z)], per=3.0)
