"""Look into the cavity with the eye-in-hand camera (hand horizontal pointing +y) and report white points."""
import sys, subprocess
from common import *
c = Ctl("peek"); r, sc = c.r, c.sc
scene_no_white(sc)
Rf = R_from_axes(np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]))
P0 = np.array([float(a) for a in sys.argv[1:4]]) if len(sys.argv) >= 4 else np.array([-0.04, 0.02, 1.03])
q = c.ik_valid(P0, Rf, seed=r.arm_q(), tries=8)
assert q is not None and c.goto_q(q, t=5.0), "look move failed"
r.snap("robot0_eye_in_hand")
subprocess.run(["python3", "cloud.py", "robot0_eye_in_hand"], capture_output=True)
d = np.load("robot0_eye_in_hand_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
m = (col.min(1) > 140) & (np.ptp(col, 1) < 30) & (pw[:, 2] > 0.905) & (pw[:, 2] < 1.1) & (pw[:, 0] > -0.16) & (pw[:, 0] < 0.07) & (pw[:, 1] > 0.2) & (pw[:, 1] < 0.45)
P = pw[m]; print("white pts in cavity region", len(P))
if len(P):
    print("x", np.round(np.percentile(P[:, 0], [1, 50, 99]), 4), "y", np.round(np.percentile(P[:, 1], [1, 50, 99]), 4), "z", np.round(np.percentile(P[:, 2], [1, 50, 99]), 4))
    top = P[P[:, 2] > np.percentile(P[:, 2], 99) - 0.008]
    print("top ring centre", np.round(top[:, :2].mean(0), 4), "n", len(top))
    for z0 in np.arange(0.94, 1.08, 0.02):
        s = P[(P[:, 2] >= z0) & (P[:, 2] < z0 + 0.02)]
        if len(s) > 5: print(f"  z[{z0:.2f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
