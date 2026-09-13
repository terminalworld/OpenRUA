#!/usr/bin/env python3
"""Cluster saved world point cloud (<cam>_P.npy) in world XY grid."""
import sys
import numpy as np
from scipy import ndimage

cam = sys.argv[1]
table_z = float(sys.argv[2]) if len(sys.argv) > 2 else 0.43
hmin = float(sys.argv[3]) if len(sys.argv) > 3 else 0.015
res = 0.005
P = np.load(f"{cam}_P.npy")
pts = P.reshape(-1, 3)
pts = pts[np.isfinite(pts).all(1)]
sel = (pts[:, 2] > table_z + hmin) & (pts[:, 2] < table_z + 0.4) & \
      (pts[:, 0] > -0.45) & (pts[:, 0] < 0.5) & (abs(pts[:, 1]) < 0.6)
pts = pts[sel]
x0, y0 = pts[:, 0].min(), pts[:, 1].min()
ix = ((pts[:, 0] - x0) / res).astype(int)
iy = ((pts[:, 1] - y0) / res).astype(int)
grid = np.zeros((ix.max() + 1, iy.max() + 1), bool)
grid[ix, iy] = True
grid = ndimage.binary_closing(grid, np.ones((3, 3)))
lab, n = ndimage.label(grid, structure=np.ones((3, 3)))
cell_lab = lab[ix, iy]
for i in range(1, n + 1):
    q = pts[cell_lab == i]
    if len(q) < 40:
        continue
    c = q.mean(0); lo = q.min(0); hi = q.max(0)
    print(f"[{i:2d}] n={len(q):5d} center=({c[0]:.3f},{c[1]:.3f}) "
          f"x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] "
          f"ztop={hi[2]:.3f} size=({hi[0]-lo[0]:.3f},{hi[1]-lo[1]:.3f})")
