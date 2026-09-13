"""Top-down eye-in-hand scan of the white mug: prints top centre, radius profile and handle direction."""
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
zt = np.percentile(P[:, 2], 99); top = P[P[:, 2] > zt - 0.006]
cx, cy = top[:, 0].mean(), top[:, 1].mean(); print("top z", round(zt, 4), "top centre", round(cx, 4), round(cy, 4), "n", len(top))
for z0 in np.arange(0.91, zt, 0.01):
    s = P[(P[:, 2] >= z0) & (P[:, 2] < z0 + 0.01)]
    if len(s) > 5:
        rr = np.hypot(s[:, 0] - cx, s[:, 1] - cy); print(f"  z[{z0:.2f}] n={len(s)} r p50={np.percentile(rr,50):.3f} p90={np.percentile(rr,90):.3f} max={rr.max():.3f}")
rr = np.hypot(P[:, 0] - cx, P[:, 1] - cy); H = P[rr > 0.055]
if len(H) > 10:
    ang = np.degrees(np.arctan2(H[:, 1] - cy, H[:, 0] - cx))
    print("handle pts", len(H), "angle p5/50/95", np.round(np.percentile(ang, [5, 50, 95]), 1), "r p50/95", np.round(np.percentile(rr[rr > 0.055], [50, 95]), 3), "z", np.round(np.percentile(H[:, 2], [5, 95]), 3))
