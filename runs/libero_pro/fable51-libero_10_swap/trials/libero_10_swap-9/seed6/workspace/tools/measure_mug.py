#!/usr/bin/env python3
"""Fit the yellow mug rim circle and handle bar from the agentview depth cloud.
usage: measure_mug.py [x_lo x_hi y_lo y_hi]  (search window, world). Prints CENTRE/BAR."""
import sys, numpy as np
sys.path.insert(0, "/workspace/tools")
import cams

def fit_circle(P):
    x, y = P[:, 0], P[:, 1]
    A = np.stack([x, y, np.ones_like(x)], 1); b = x * x + y * y
    c = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = c[0] / 2, c[1] / 2
    return cx, cy, np.sqrt(c[2] + cx * cx + cy * cy)

def main():
    win = list(map(float, sys.argv[1:5])) if len(sys.argv) >= 5 else [-0.12, 0.12, 0.0, 0.30]
    d = np.load("/workspace/snaps/agentview_depth.npy")
    P = cams.cloud("agentview", d).reshape(-1, 3)
    m = (P[:, 0] > win[0]) & (P[:, 0] < win[1]) & (P[:, 1] > win[2]) & (P[:, 1] < win[3]) & (P[:, 2] > 0.905) & (P[:, 2] < 1.02)
    P = P[m]
    rim = P[P[:, 2] > 0.995]
    cx, cy, r = fit_circle(rim[:, :2])
    print(f"rim pts {len(rim)} circle centre ({cx:.4f},{cy:.4f}) r {r:.4f} z max {P[:,2].max():.3f}")
    # handle = points outside the body cylinder (r>0.06 from the centre), z 0.93-0.975
    dist = np.hypot(P[:, 0] - cx, P[:, 1] - cy)
    H = P[(dist > 0.062) & (dist < 0.11) & (P[:, 2] > 0.93) & (P[:, 2] < 0.975)]
    if len(H):
        ang = np.degrees(np.arctan2(H[:, 1] - cy, H[:, 0] - cx))
        print(f"handle pts {len(H)} x[{H[:,0].min():.3f},{H[:,0].max():.3f}] y[{H[:,1].min():.3f},{H[:,1].max():.3f}] "
              f"dist[{dist[(dist>0.062)&(dist<0.11)&(P[:,2]>0.93)&(P[:,2]<0.975)].min():.3f},...] angle median {np.median(ang):.1f} deg")
        bx, by = np.median(H[:, 0]), np.median(H[:, 1])
        print(f"BAR ({bx:.4f},{by:.4f})  CENTRE ({cx:.4f},{cy:.4f})")
    else:
        print("no handle points found")

if __name__ == "__main__":
    main()
