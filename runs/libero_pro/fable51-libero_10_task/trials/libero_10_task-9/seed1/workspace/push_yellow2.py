"""Push the yellow mug toward -y with a 20deg forward-pitched hand (wrist stays
closer to the base and above/behind the door). Dense waypoints + tight jump
check so the joint-space path cannot swing into the microwave."""
import numpy as np
from rob import Robot, approach_quat
r = Robot("pushy2")
TH = np.deg2rad(20)
qn = approach_quat([np.sin(TH), 0, -np.cos(TH)], [1, 0, 0])
X, Z = 0.12, 0.965

def go(pts, per=2.0, corrected_last=True, jump=0.35):
    qs = []; seed = r.arm_q()
    for p in pts:
        q = r.ik(np.array(p), qn, seed=seed, timeout=3.0)
        if q is None: raise SystemExit(f"IK failed at {p}")
        prev = np.array(qs[-1]) if qs else np.array(seed)
        if np.max(np.abs(np.array(q) - prev)) > jump:
            raise SystemExit(f"joint jump at {p}: {np.round(np.array(q)-prev,2)}")
        qs.append(q); seed = q
    r.move_traj(qs, [per * (i + 1) for i in range(len(qs))])
    if corrected_last: r.move_q_corrected(qs[-1], seconds=2.0, iters=2)
    p, qq = r.tcp(); print("TCP", p.round(4), "q", np.round(r.arm_q(), 2))

def line(a, b, step):
    a, b = np.array(a, float), np.array(b, float)
    n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
    return [tuple(a + (b - a) * k / n) for k in range(1, n + 1)]

r.gripper(0.0)
p0 = r.tcp()[0]
print("-- reorient/travel to start (high)")
go(line(p0, (X, 0.19, 1.20), 0.06), per=1.5, jump=1.0)
print("-- descend"); go(line((X, 0.19, 1.20), (X, 0.19, Z), 0.04), per=1.2)
print("-- push -y"); go(line((X, 0.19, Z), (X, -0.12, Z), 0.04), per=1.2, corrected_last=False)
print("-- lift");    go(line((X, -0.12, Z), (X, -0.12, 1.20), 0.06), per=1.2, corrected_last=False)
