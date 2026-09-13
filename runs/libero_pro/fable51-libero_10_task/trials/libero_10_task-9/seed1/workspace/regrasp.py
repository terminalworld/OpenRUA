import numpy as np
from rob import Robot
r = Robot("regrasp")
qn = np.load("snaps/quat_push.npy")   # 30 deg pitched-down approach, fingers along x
def go(pts, per=3.0, iters=2):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    r.move_q_corrected(qs[-1], seconds=2.0, iters=iters)
    p, qq = r.tcp(); print("TCP", p.round(4), "fingers", np.round(r.fingers(), 4))
X, Z = -0.042, 0.982
print("-- open"); r.gripper(0.04)
print("-- approach"); go([(X, 0.16, 1.03), (X, 0.22, Z), (X, 0.275, Z)], per=3.0)
print("-- pinch"); f = r.gripper(0.0)
