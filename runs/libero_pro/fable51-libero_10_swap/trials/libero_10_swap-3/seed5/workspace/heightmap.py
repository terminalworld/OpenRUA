#!/usr/bin/env python3
"""Merged height map (cm above table z=0.9) from saved depth/T/K npy files.
Usage: python3 heightmap.py cam1[,cam2,...] x0 x1 y0 y1 [step=0.02]
"""
import sys
import numpy as np

cams = sys.argv[1].split(",")
x0, x1, y0, y1 = map(float, sys.argv[2:6])
step = float(sys.argv[6]) if len(sys.argv) > 6 else 0.02
pts = []
for cam in cams:
    d = np.load(f"{cam}_depth.npy"); T = np.load(f"{cam}_T.npy"); K = np.load(f"{cam}_K.npy")
    H, W = d.shape; fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    v, u = np.mgrid[0:H, 0:W]
    ok = np.isfinite(d) & (d > 0)
    X = (u - cx) * d / fx; Y = (v - cy) * d / fy
    P = np.stack([X, Y, d, np.ones_like(d)], -1)[ok] @ T.T
    pts.append(P[:, :3])
P = np.concatenate(pts)
xs = np.arange(x0, x1 + 1e-9, step); ys = np.arange(y0, y1 + 1e-9, step)
print("rows = world x (cm), cols = world y (cm); value = max height cm above table (0.9)")
print("      " + " ".join(f"{y*100:4.0f}" for y in ys))
for x in xs:
    row = []
    for y in ys:
        m = (np.abs(P[:, 0] - x) < step / 2) & (np.abs(P[:, 1] - y) < step / 2)
        row.append(f"{(P[m, 2].max() - 0.9) * 100:4.0f}" if m.any() else "   .")
    print(f"{x*100:5.0f} " + " ".join(row))
