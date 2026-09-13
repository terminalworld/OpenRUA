"""Locate objects above the table from a camera cloud: cluster points with z>0.94 by xy grid."""
import sys, numpy as np
from cloud import cloud
cam = sys.argv[1] if len(sys.argv) > 1 else "sideview"
X, Y, Z, _ = cloud(cam)
m = (Z > 0.945) & (Z < 1.30) & (X > -0.45) & (X < 0.45) & (Y > -0.45) & (Y < 0.45)
X, Y, Z = X[m], Y[m], Z[m]
# 2 cm grid occupancy summary
gx = np.round(X / 0.02).astype(int); gy = np.round(Y / 0.02).astype(int)
cells = {}
for a, b, z in zip(gx, gy, Z):
    k = (a, b); cells.setdefault(k, []).append(z)
rows = sorted(((k[0]*0.02, k[1]*0.02, len(v), max(v)) for k, v in cells.items()), key=lambda r: (-r[2]))
print("cells with >=5 points (x, y, n, zmax):")
for r in rows:
    if r[2] >= 5: print("  %.2f %.2f  n=%3d  zmax=%.3f" % r)
