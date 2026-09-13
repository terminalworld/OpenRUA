#!/usr/bin/env python3
"""Fit the axis of a moka pot lying on the table from the eye-in-hand cloud (ridge fit).
Usage: python3 fitpot.py <cx> <cy> <ux> <uy>   (initial guess of axis centre and direction)
Prints refined centre/direction, ridge height and the body extent along the axis."""
import sys
import numpy as np

C = np.load("robot0_eye_in_hand_cloud.npy").reshape(-1, 3)
C = C[np.isfinite(C).all(1)]
c = np.array([float(sys.argv[1]), float(sys.argv[2])])
u = np.array([float(sys.argv[3]), float(sys.argv[4])]); u /= np.linalg.norm(u)
m = (C[:, 2] > 0.905) & (C[:, 2] < 1.05) & (np.linalg.norm(C[:, :2] - c, axis=1) < 0.2)
P = C[m]
for it in range(4):
    n = np.array([-u[1], u[0]])
    t = (P[:, :2] - c) @ u; w = (P[:, :2] - c) @ n
    pts = []
    for a in np.arange(-0.09, 0.07, 0.005):
        s = (t >= a) & (t < a + 0.005) & (np.abs(w) < 0.05)
        if s.sum() < 20:
            continue
        zz = P[s, 2]; k = zz > zz.max() - 0.003
        pts.append([t[s][k].mean(), w[s][k].mean(), zz[k].mean()])
    pts = np.array(pts)
    A = np.c_[pts[:, 0], np.ones(len(pts))]
    slope, off = np.linalg.lstsq(A, pts[:, 1], rcond=None)[0]
    c = c + off * n
    u = u + slope * n; u /= np.linalg.norm(u)
n = np.array([-u[1], u[0]])
t = (P[:, :2] - c) @ u; w = (P[:, :2] - c) @ n
body = (np.abs(w) < 0.045) & (P[:, 2] > 0.955)
print(f"centre {c.round(4)} u {u.round(4)} ridge z {pts[:,2].mean():.4f} resid {np.abs(pts[:,1]-A@[slope,off]).max():.4f}")
print(f"body t range {t[body].min():+.3f} .. {t[body].max():+.3f}")
for a in np.arange(-0.14, 0.14, 0.01):
    s = (t >= a) & (t < a + 0.01) & (np.abs(w) < 0.05)
    if s.sum() > 5:
        print(f"t {a:+.2f} n {s.sum():4d} zmax {P[s,2].max():.3f} w {w[s].min():+.3f}..{w[s].max():+.3f}")
