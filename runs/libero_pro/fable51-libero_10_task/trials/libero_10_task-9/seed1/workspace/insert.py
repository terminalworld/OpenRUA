import numpy as np, rclpy
from rob import Robot
r = Robot("insert")
qn = np.load("snaps/quat_ins.npy")
Z = 0.997; X = -0.061
def go(pts, per=3.0, corrected_last=True):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed)
        if q is None: raise SystemExit(f"IK failed at {p}")
        qs.append(q); seed = q
    times = [per * (i + 1) for i in range(len(qs))]
    r.move_traj(qs, times)
    if corrected_last: r.move_q_corrected(qs[-1], seconds=2.0)
    p, qq = r.tcp(); print("TCP", p.round(4), "quat", qq.round(3), "fingers", np.round(r.fingers(), 4))
print("-- descend")
go([(-0.065, 0.13, 1.10), (X, 0.13, Z)], per=3.0)
print("-- insert")
go([(X, 0.18, Z), (X, 0.23, Z), (X, 0.27, Z), (X, 0.31, Z)], per=2.5)
