#!/usr/bin/env python3
"""Birdview height map (cm above the table) of a region.
  python3 tools/heightmap.py x0 x1 y0 y1 [cell_m]"""
import sys, subprocess, os
import numpy as np
sys.path.insert(0, "/workspace/tools")
import cams

x0, x1, y0, y1 = map(float, sys.argv[1:5])
cell = float(sys.argv[5]) if len(sys.argv) > 5 else 0.01
os.makedirs("/workspace/snaps", exist_ok=True)
subprocess.run(["python3", "/workspace/tools/cam_snap.py", "birdview", "--depth"], cwd="/workspace/snaps",
               check=True, stdout=subprocess.DEVNULL)
depth = np.load("/workspace/snaps/birdview_depth.npy")
pts = cams.cloud("birdview", depth).reshape(-1, 3)
m = (pts[:, 0] >= x0) & (pts[:, 0] < x1) & (pts[:, 1] >= y0) & (pts[:, 1] < y1) & (pts[:, 2] < 1.3)
pts = pts[m]
nx, ny = int(round((x1 - x0) / cell)), int(round((y1 - y0) / cell))
H = np.full((nx, ny), -1.0)
ix = ((pts[:, 0] - x0) / cell).astype(int); iy = ((pts[:, 1] - y0) / cell).astype(int)
for i, j, z in zip(ix, iy, pts[:, 2]):
    H[i, j] = max(H[i, j], z)
print("rows x from %.3f, cols y from %.3f, cell %.3f; values cm above table" % (x0, y0, cell))
print("      " + " ".join("%3d" % round((y0 + (j + 0.5) * cell) * 100) for j in range(ny)))
for i in range(nx):
    row = " ".join("%3d" % round((H[i, j] - 0.90) * 100) if H[i, j] > 0 else "  ." for j in range(ny))
    print("%6.3f " % (x0 + (i + 0.5) * cell) + row)
