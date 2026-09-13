#!/usr/bin/env python3
"""Lift clear of the can, then measure its centre from silhouette edges.

agentview looks along -x -> sees the can's +-y edges; sideview/frontview see
+-x edges. Midpoint of the edges beats a centroid (partial rim, occlusion).
"""
import sys

from arm import *
from seg import cloud, region

Q_SOUP = q_grasp(np.pi / 2, np.radians(30))
Z_HOVER = 0.62

a = Arm()
p, _ = a.hand_pose()
print("tcp now", np.round(p, 4))
if p[2] < Z_HOVER - 0.02:
    a.move_to([p[0], p[1], Z_HOVER], q=Q_SOUP, seconds=4, steps=2)

XR, YR, ZR = (-0.27, -0.14), (-0.20, -0.06), (0.435, 0.512)  # can body only, below rim
est = {}
for cam in ("agentview", "sideview", "frontview", "birdview"):
    P = cloud(cam, a.node)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = (X > XR[0]) & (X < XR[1]) & (Y > YR[0]) & (Y < YR[1]) & (Z > ZR[0]) & (Z < ZR[1])
    if m.sum() < 20:
        print(cam, "sees nothing")
        continue
    xs, ys = X[m], Y[m]
    # robust edges: 1st/99th percentile
    xe, ye = np.percentile(xs, [1, 99]), np.percentile(ys, [1, 99])
    est[cam] = dict(n=int(m.sum()), xmid=xe.mean(), ymid=ye.mean(), dx=xe[1] - xe[0], dy=ye[1] - ye[0])
    print(f"{cam}: n={m.sum()} x=[{xe[0]:.4f},{xe[1]:.4f}] (w {xe[1]-xe[0]:.3f})  "
          f"y=[{ye[0]:.4f},{ye[1]:.4f}] (w {ye[1]-ye[0]:.3f})  ztop={Z[m].max():.4f}")
