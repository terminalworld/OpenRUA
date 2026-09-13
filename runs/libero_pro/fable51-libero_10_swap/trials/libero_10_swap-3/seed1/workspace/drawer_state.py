#!/usr/bin/env python3
"""Report the drawer front-face y position from fresh camera frames.
Front face = max-y of points with x in [-0.195,-0.155] (beside the handle),
z in [0.92,0.98] (the drawer front, below the hand body)."""
import subprocess, sys
import numpy as np
cams = sys.argv[1:] or ["sideview", "agentview"]
for cam in cams:
    subprocess.run([sys.executable, "pc.py", cam], check=True, capture_output=True)
    P = np.load(f"{cam}_xyz.npy").reshape(-1, 3)
    P = P[np.isfinite(P[:, 0])]
    for xr in [(-0.195, -0.155), (-0.06, -0.02)]:
        f = P[(P[:, 0] > xr[0]) & (P[:, 0] < xr[1]) & (P[:, 2] > 0.92) & (P[:, 2] < 0.98) & (P[:, 1] > -0.3) & (P[:, 1] < 0.005)]
        if len(f):
            ys = np.sort(f[:, 1])
            print(f"{cam} x{xr}: front-face y ~ {np.percentile(ys, 98):.3f} (max {ys.max():.3f}), n={len(f)}")
        else:
            print(f"{cam} x{xr}: no points")
    h = P[(P[:, 0] > -0.14) & (P[:, 0] < -0.06) & (P[:, 2] > 0.93) & (P[:, 2] < 0.975) & (P[:, 1] > -0.3) & (P[:, 1] < 0.005)]
    if len(h):
        print(f"{cam} handle-bar region: max y {np.percentile(h[:,1],98):.3f}")
