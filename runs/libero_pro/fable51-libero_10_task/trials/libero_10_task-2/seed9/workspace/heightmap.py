#!/usr/bin/env python3
"""Height map from birdview depth: camera at world (-0.2, 0, 3.0) looking
straight down; optical x -> world -y ... derived from TF:
R = [[0,1,0],[1,0,0],[0,0,-1]] (from tf2_echo matrix first row 0 1 0).
"""
import sys
import numpy as np

fx = fy = 579.4112549695428
cx, cy = 320.0, 240.0
cam_t = np.array([-0.2, 0.0, 3.0])
# quaternion (0.707,0.707,0,0) -> R
x, y, z, w = 0.7071068, 0.7071068, 0.0, 0.0
R = np.array([
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
])

d = np.load(sys.argv[1] if len(sys.argv) > 1 else "birdview_depth.npy")
H, W = d.shape
vs, us = np.mgrid[0:H, 0:W]
pc = np.stack([(us - cx) * d / fx, (vs - cy) * d / fy, d], -1)
pw = pc @ R.T + cam_t
X, Y, Z = pw[..., 0], pw[..., 1], pw[..., 2]

def region(name, mask):
    if not mask.any():
        print(name, "empty"); return
    print(f"{name}: n={mask.sum()} x[{X[mask].min():.3f},{X[mask].max():.3f}] "
          f"y[{Y[mask].min():.3f},{Y[mask].max():.3f}] z[{Z[mask].min():.3f},{Z[mask].max():.3f}] "
          f"centroid=({X[mask].mean():.3f},{Y[mask].mean():.3f}) ztop={np.percentile(Z[mask],95):.3f}")

table = np.abs(Z - 0.90) < 0.005
print("table z median", np.median(Z[table]))
above = (Z > 0.905) & (X > -0.35) & (X < 0.4) & (np.abs(Y) < 0.5)
# cluster by rough y bands
region("pan region (y<-0.12)", above & (Y < -0.12) & (Y > -0.4) & (X < 0.1))
region("pan handle band (-0.13<y<-0.02, x<0)", above & (Y > -0.13) & (Y < -0.02) & (X < 0.0) & (X > -0.12))
region("moka region", above & (Y > -0.1) & (Y < 0.08) & (X > 0.0))
region("stove region", above & (Y > 0.1) & (Y < 0.3) & (X > -0.15) & (X < 0.1))
region("knob region", above & (Y > 0.1) & (Y < 0.3) & (X < -0.15) & (X > -0.3))
region("stove top plate (z>0.92)", above & (Y > 0.1) & (Y < 0.3) & (X > -0.15) & (X < 0.1) & (Z > 0.925))
region("stove burner (z>0.935)", above & (Y > 0.1) & (Y < 0.3) & (X > -0.15) & (X < 0.1) & (Z > 0.935))
np.save("heightZ.npy", Z); np.save("worldX.npy", X); np.save("worldY.npy", Y)
