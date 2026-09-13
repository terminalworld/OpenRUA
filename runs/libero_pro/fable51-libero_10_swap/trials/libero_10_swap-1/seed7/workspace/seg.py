#!/usr/bin/env python3
"""Segment objects above the table in a grab.py npz; print world centroid/extents.
Usage: seg.py <prefix> [min_h=0.01]"""
import sys
import numpy as np, cv2

d = np.load(sys.argv[1] + ".npz")
K, T, dep, img = d["K"], d["T"], d["depth"], d["img"]
min_h = float(sys.argv[2]) if len(sys.argv) > 2 else 0.01
TABLE = 0.4249
H, W = dep.shape
vs, us = np.mgrid[0:H, 0:W]
z = dep
X = (us - K[0, 2]) * z / K[0, 0]
Y = (vs - K[1, 2]) * z / K[1, 1]
P = np.stack([X, Y, z, np.ones_like(z)], -1) @ T.T
wz = P[..., 2]
mask = (np.isfinite(z) & (wz > TABLE + min_h) & (wz < TABLE + 0.35)).astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30:
        continue
    m = lab == i
    pts = P[m][:, :3]
    col = img[m].mean(0)[::-1]
    x0, y0, w, h = stats[i, :4]
    print(f"comp{i}: px bbox u[{x0},{x0+w}] v[{y0},{y0+h}] area={stats[i,4]} "
          f"cent={cents[i].round(0)} world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] "
          f"y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} "
          f"mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) rgb={col.round(0)}")
