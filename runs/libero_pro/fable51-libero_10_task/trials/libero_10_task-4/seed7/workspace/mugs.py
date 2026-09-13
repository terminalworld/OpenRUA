#!/usr/bin/env python3
"""Fit rim circles to the mugs in birdview_world.npy (run scan.py first)."""
import numpy as np, cv2, sys
P = np.load("birdview_world.npy"); z = P[..., 2]
color = cv2.imread("birdview.png")
TABLE = 0.425
regions = {"white": (-0.088, -0.147), "yellow": (-0.029, 0.117), "red": (-0.216, -0.002)}
for name, (cx0, cy0) in regions.items():
    near = (np.abs(P[..., 0] - cx0) < 0.09) & (np.abs(P[..., 1] - cy0) < 0.09) & np.isfinite(z)
    obj = near & (z > TABLE + 0.03) & (z < 0.7)
    pts = P[obj]
    ztop = pts[:, 2].max()
    # rim: highest 2 cm
    rim = pts[pts[:, 2] > ztop - 0.02]
    # algebraic circle fit x^2+y^2+Dx+Ey+F=0
    A = np.c_[rim[:, 0], rim[:, 1], np.ones(len(rim))]
    b = -(rim[:, 0] ** 2 + rim[:, 1] ** 2)
    D, E, F = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = -D / 2, -E / 2; r = np.sqrt(cx * cx + cy * cy - F)
    # handle: object pixels farther than r+1.5cm from center
    d = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
    h = pts[d > r + 0.012]
    hdir = None
    if len(h):
        hv = h[:, :2].mean(0) - [cx, cy]; hdir = np.degrees(np.arctan2(hv[1], hv[0]))
    print(f"{name}: rim center=({cx:.4f},{cy:.4f}) r={r:.4f} (diam {2*r*100:.1f}cm) ztop={ztop:.3f} "
          f"height={ztop-TABLE:.3f} n_rim={len(rim)} handle_dir={None if hdir is None else round(hdir)}deg n_handle={len(h)} "
          f"body_z_range=[{pts[:,2].min():.3f},{ztop:.3f}]")
    # z profile of the mug: extent in xy at several heights
    for zl in np.arange(TABLE + 0.03, ztop, 0.02):
        s = pts[(pts[:, 2] > zl) & (pts[:, 2] <= zl + 0.02)]
        if len(s):
            dd = np.hypot(s[:, 0] - cx, s[:, 1] - cy)
            print(f"   z {zl:.3f}-{zl+0.02:.3f}: n={len(s)} maxdist={dd.max():.3f} mediandist={np.median(dd):.3f}")
