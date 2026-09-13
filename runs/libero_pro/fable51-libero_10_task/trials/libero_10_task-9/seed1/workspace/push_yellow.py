"""Push the fallen yellow mug out of the door sweep: top-down closed fingertips
on its +y side, then slide it toward -y (away from the microwave)."""
import numpy as np
from rob import Robot, topdown_quat
r = Robot("pushy")
qn = topdown_quat([1, 0, 0])   # fingers side by side along x, palm thin along y
X, Z = 0.09, 0.965

def go(pts, per=3.0, corrected_last=True):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed, timeout=3.0)
        if q is None: raise SystemExit(f"IK failed at {p}")
        if qs and np.max(np.abs(np.array(q) - np.array(qs[-1]))) > 0.8:
            raise SystemExit(f"joint jump at {p}: {np.round(np.array(q)-np.array(qs[-1]),2)}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    if corrected_last: r.move_q_corrected(qs[-1], seconds=2.0, iters=2)
    p, qq = r.tcp(); print("TCP", p.round(4), "q", np.round(r.arm_q(), 2))

r.gripper(0.0)
print("-- over start"); go([(X, 0.19, 1.25)], per=4.0)
print("-- descend");    go([(X, 0.19, 1.05), (X, 0.19, Z)], per=3.0)
print("-- push -y");    go([(X, 0.12, Z), (X, 0.04, Z), (X, -0.04, Z), (X, -0.11, Z)], per=2.5, corrected_last=False)
print("-- lift");       go([(X, -0.11, 1.20)], per=3.0, corrected_last=False)
