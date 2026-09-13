"""Top-down eye-in-hand scan of the lying white mug: bar (topmost points) centre/extent and body extents."""
import sys, subprocess
from common import *
x, y = float(sys.argv[1]), float(sys.argv[2])
c = Ctl("scan"); r, sc = c.r, c.sc
scene_no_white(sc)
q = c.ik_valid(np.array([x, y, 1.32]), R_DOWN, seed=r.arm_q(), tries=8)
assert q is not None and c.goto_q(q, t=5.0), "look move failed"
r.snap("robot0_eye_in_hand")
subprocess.run(["python3", "cloud.py", "robot0_eye_in_hand"], capture_output=True)
d = np.load("robot0_eye_in_hand_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
m = (col.min(1) > 140) & (np.ptp(col, 1) < 30) & (pw[:, 2] > 0.905) & (pw[:, 2] < 1.1) & (np.abs(pw[:, 0] - x) < 0.15) & (np.abs(pw[:, 1] - y) < 0.15)
P = pw[m]; print("pts", len(P))
print("x range", np.round(np.percentile(P[:, 0], [1, 99]), 4), "y range", np.round(np.percentile(P[:, 1], [1, 99]), 4), "z max", round(P[:, 2].max(), 4))
zt = np.percentile(P[:, 2], 99.5)
for dz in (0.006, 0.012, 0.02):
    top = P[P[:, 2] > zt - dz]
    print(f"top-{dz}: n={len(top)} centre {top[:,0].mean():.4f} {top[:,1].mean():.4f} x[{top[:,0].min():.3f},{top[:,0].max():.3f}] y[{top[:,1].min():.3f},{top[:,1].max():.3f}]")
for z0 in np.arange(0.91, zt, 0.01):
    s = P[(P[:, 2] >= z0) & (P[:, 2] < z0 + 0.01)]
    if len(s) > 5:
        print(f"  z[{z0:.2f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
