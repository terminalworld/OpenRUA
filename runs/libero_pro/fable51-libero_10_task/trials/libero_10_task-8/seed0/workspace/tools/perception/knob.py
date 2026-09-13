#!/usr/bin/env python3
"""Locate the moka-pot lid knob near (x0,y0) from the eye-in-hand cloud.
Usage: knob.py x0 y0 -> prints knob center xyz, lid top z, and TCP delta."""
import sys, subprocess, numpy as np
x0, y0 = float(sys.argv[1]), float(sys.argv[2])
subprocess.run(["python3", "tools/perception/depth_world.py", "robot0_eye_in_hand"], check=True, timeout=120)
xyz = np.load("img/robot0_eye_in_hand_xyz.npy"); x, y, z = xyz[..., 0], xyz[..., 1], xyz[..., 2]
sel = (np.abs(x - x0) < 0.03) & (np.abs(y - y0) < 0.03) & (z > 1.0) & (z < 1.1)
p = xyz[sel]
ztop = p[:, 2].max()
knob = p[p[:, 2] > ztop - 0.008]
lid = p[(p[:, 2] > 1.03) & (p[:, 2] < ztop - 0.012)]
print(f"knob n={len(knob)} center=({knob[:,0].mean():.4f}, {knob[:,1].mean():.4f}) "
      f"x[{knob[:,0].min():.4f},{knob[:,0].max():.4f}] y[{knob[:,1].min():.4f},{knob[:,1].max():.4f}] "
      f"ztop={ztop:.4f}  lid z max={lid[:,2].max() if len(lid) else float('nan'):.4f} n={len(lid)}")
