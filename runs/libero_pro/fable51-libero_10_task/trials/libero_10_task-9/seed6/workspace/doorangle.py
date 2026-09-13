"""Measure the microwave door angle from the birdview heightmap (run heightmap.py first or pass --fresh)."""
import sys, subprocess, numpy as np
if "--fresh" in sys.argv: subprocess.run(["python3", "heightmap.py"], capture_output=True)
P = np.load("birdview_world.npy")
H = np.array([-0.165, 0.27])
X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
# door top: tall thin object, z in [1.09, 1.12], west of the hinge region or in front of the cavity, not the arm (x > -0.35)
m = (Z > 1.085) & (Z < 1.125) & (X > -0.36) & (X < 0.16) & (Y < 0.262) & (Y > -0.1)
import cv2
n, lab, st, ce = cv2.connectedComponentsWithStats(m.astype(np.uint8))
# door = component whose points touch the hinge neighbourhood (within 6 cm of H)
best = None
for i in range(1, n):
    if st[i, 4] < 30: continue
    p = P[lab == i][:, :2]
    if np.linalg.norm(p - H, axis=1).min() < 0.06 and (best is None or st[i, 4] > st[best, 4]): best = i
pts = P[lab == best][:, :2] if best is not None else np.zeros((0, 2))
print("door-top points:", len(pts))
if len(pts) < 20: sys.exit()
c = pts.mean(0); u, s, vt = np.linalg.svd(pts - c); d = vt[0]
if d @ (c - H) < 0: d = -d
th = np.degrees(np.arctan2(d[1], d[0]))
rho = (pts - H) @ d
print(f"door dir {np.round(d,3)} theta={th:.1f} deg  centroid {np.round(c,3)}  rho range [{rho.min():.3f},{rho.max():.3f}]  perp spread {np.std((pts-H)@np.array([-d[1],d[0]])):.3f}")
n1 = np.array([-d[1], d[0]]); off = (pts - H) @ n1
print(f"perp offset of top points about H: mean {off.mean():.4f} min {off.min():.4f} max {off.max():.4f}")
