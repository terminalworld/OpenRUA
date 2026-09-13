#!/usr/bin/env python3
"""Snapshot birdview and report mug / door / robot positions."""
import subprocess, sys
import numpy as np
subprocess.run([sys.executable, "/workspace/cloud.py", "birdview", "/workspace/bird.npz"], check=True, capture_output=True)
d = np.load("/workspace/bird.npz"); X, Y, Z = d["X"], d["Y"], d["Z"]
ok = np.isfinite(Z)

def region(name, xr, yr, zr=(0.905, 1.03)):
    m = ok & (X > xr[0]) & (X < xr[1]) & (Y > yr[0]) & (Y < yr[1]) & (Z > zr[0]) & (Z < zr[1])
    if m.sum() < 5:
        print(f"{name}: not found"); return None
    mr = m & (Z > Z[m].max() - 0.012)   # rim points
    cx, cy = (X[mr].min() + X[mr].max()) / 2, (Y[mr].min() + Y[mr].max()) / 2
    print(f"{name}: rim center=({cx:.3f},{cy:.3f}) z=[{Z[m].min():.3f},{Z[m].max():.3f}] "
          f"x=[{X[m].min():.3f},{X[m].max():.3f}] y=[{Y[m].min():.3f},{Y[m].max():.3f}] n={m.sum()}")
    # handle = points outside rim circle radius 0.05
    mh = m & (np.hypot(X - cx, Y - cy) > 0.052)
    if mh.sum() > 5:
        hx, hy = X[mh].mean(), Y[mh].mean()
        ang = np.degrees(np.arctan2(hy - cy, hx - cx))
        print(f"   handle: mean=({hx:.3f},{hy:.3f}) dir={ang:.0f}deg (0=+x, -90=-y) z=[{Z[mh].min():.3f},{Z[mh].max():.3f}] n={mh.sum()}")
    return cx, cy

yl = region("yellow mug", (-0.3, 0.3), (-0.12, 0.25))
gr = region("gray mug", (-0.2, 0.3), (0.25, 0.5))
# door: points with y<-0.35 near x~-0.25, z>0.95
m = ok & (Y < -0.355) & (Y > -0.75) & (Z > 0.95) & (Z < 1.12) & (X > -0.45) & (X < 0.15)
if m.sum() > 20:
    print(f"door(open part): x=[{X[m].min():.3f},{X[m].max():.3f}] y=[{Y[m].min():.3f},{Y[m].max():.3f}] n={m.sum()}")
    for y0 in (-0.40, -0.50, -0.58):
        mm = m & (np.abs(Y - y0) < 0.01)
        if mm.sum(): print(f"   y={y0}: x=[{X[mm].min():.3f},{X[mm].max():.3f}]")
else:
    print("door: nothing beyond y<-0.355 (closed?)  n=", m.sum())
# front face of microwave region
m = ok & (Y < -0.33) & (Y > -0.40) & (Z > 0.95) & (Z < 1.12) & (X > -0.3) & (X < 0.1)
print(f"front strip y[-0.40,-0.33]: n={m.sum()} y=[{Y[m].min() if m.sum() else 0:.3f},{Y[m].max() if m.sum() else 0:.3f}]")
