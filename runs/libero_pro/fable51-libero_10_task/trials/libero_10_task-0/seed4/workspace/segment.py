#!/usr/bin/env python3
"""Segment above-table blobs from a captured camera and print world stats.

Usage: python3 segment.py <camera> [z_table=0.425] [min_h=0.012]
"""
import sys

import cv2
import numpy as np

cam = sys.argv[1]
z_table = float(sys.argv[2]) if len(sys.argv) > 2 else 0.425
min_h = float(sys.argv[3]) if len(sys.argv) > 3 else 0.012
depth = np.load(f"{cam}_depth.npy")
m = np.load(f"{cam}_meta.npz")
K, T = m["K"], m["T"]
color = cv2.imread(f"{cam}.png")
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
z = depth
X = (uu - K[0, 2]) * z / K[0, 0]
Y = (vv - K[1, 2]) * z / K[1, 1]
P = np.stack([X, Y, z, np.ones_like(z)], -1) @ T.T
wz = P[..., 2]
mask = (wz > z_table + min_h) & (wz < z_table + 0.35) & np.isfinite(z)
# restrict to table region (world radius) to drop the robot body & walls
wx, wy = P[..., 0], P[..., 1]
mask &= (wx > -0.35) & (wx < 0.45) & (np.abs(wy) < 0.5)
n, lab = cv2.connectedComponents(mask.astype(np.uint8))
print(f"{cam}: {n-1} blobs (z_table={z_table})")
for i in range(1, n):
    sel = lab == i
    if sel.sum() < 15:
        continue
    pts = P[sel]
    bgr = color[sel].mean(0)
    us, vs = uu[sel], vv[sel]
    print(f" blob {i}: n={sel.sum():4d} px=({us.mean():.0f},{vs.mean():.0f}) "
          f"centroid=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) "
          f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] "
          f"y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
          f"ztop={np.percentile(pts[:,2],95):.3f} "
          f"rgb=({bgr[2]:.0f},{bgr[1]:.0f},{bgr[0]:.0f})")
