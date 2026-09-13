import numpy as np
from rob import Robot
r = Robot("release")
qn = np.load("snaps/quat_ins.npy")
print("-- open"); r.gripper(0.04)
X = -0.061
def go(pts, per=3.0):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    r.move_q_corrected(qs[-1], seconds=2.0, iters=2)
    p, qq = r.tcp(); print("TCP", p.round(4), "fingers", np.round(r.fingers(), 4))
print("-- retreat"); go([(X, 0.25, 1.00), (X, 0.19, 1.00)], per=2.5)
print("-- lift"); go([(X, 0.19, 1.10), (X, 0.17, 1.22)], per=2.5)
