import numpy as np
from rob import Robot, approach_quat
r = Robot("nudge5")
phi = np.deg2rad(10)
qn = approach_quat(np.array([0, np.cos(phi), -np.sin(phi)]), np.array([1.0, 0, 0]))
np.save("snaps/quat_push10.npy", qn)
def go(pts, per=3.0, iters=2):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    r.move_q_corrected(qs[-1], seconds=2.0, iters=iters)
    p, qq = r.tcp(); print("TCP", p.round(4), "fingers", np.round(r.fingers(), 4))
X = 0.0; Z = 1.050
print("-- approach"); go([(X, 0.12, 1.12), (X, 0.20, Z)], per=3.0)
print("-- push"); go([(X, 0.25, Z), (X, 0.28, Z), (X, 0.30, Z)], per=2.0)
print("-- back off"); go([(X, 0.20, Z)], per=3.0, iters=1)
