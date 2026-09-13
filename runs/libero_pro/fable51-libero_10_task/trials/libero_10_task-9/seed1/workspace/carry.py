import numpy as np
from rob import Robot
r = Robot("carry")
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
X = -0.042
print("-- lift"); go([(X, 0.275, 0.998)], per=2.5)
print("-- deeper"); go([(X, 0.295, 0.998), (X, 0.315, 0.998)], per=2.5)
print("-- lower"); go([(X, 0.315, 0.985)], per=2.5)
print("-- open"); r.gripper(0.04)
print("-- retreat"); go([(X, 0.22, 0.99), (X, 0.16, 1.03)], per=3.0, iters=1)
