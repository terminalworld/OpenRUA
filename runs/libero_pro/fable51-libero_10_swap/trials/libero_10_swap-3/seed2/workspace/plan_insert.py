#!/usr/bin/env python3
"""Grid-search a 2D (y, z, tilt) insertion path for the bowl into the drawer.

Bowl model: cone frustum, r_bottom..r_top over height H, described by the
outline in the y-z plane (x extent irrelevant: obstacles are y/z only).
Obstacles (world frame, from depth measurements):
  handle:  y >= Y_HANDLE and Z_H_LO <= z <= Z_H_HI (middle drawer handle)
  wall:    y >= Y_WALL_IN and z <= Z_WALL_TOP  (drawer front wall)
  cabinet: y <= Y_CAB (cabinet face)
  floor:   z <= Z_FLOOR
Clearance = min signed distance of bowl outline points to obstacle boxes.
"""
import numpy as np

R_BOT, R_TOP, H = 0.036, 0.056, 0.052
Y_HANDLE, Z_H_LO, Z_H_HI = -0.189, 1.005, 1.035
Y_WALL_IN, Z_WALL_TOP = -0.077, 0.9834
Y_CAB = -0.221
Z_FLOOR = 0.924


def outline(yc, zb, theta):
    """Bowl outline points (y, z) for bottom-center at (yc, zb), tilted by
    theta about x (positive = -y side down)."""
    hs = np.linspace(0, H, 14)
    rs = R_BOT + (R_TOP - R_BOT) * hs / H
    pts = np.concatenate([np.c_[-rs, hs], np.c_[rs, hs],
                          np.c_[np.linspace(-R_BOT, R_BOT, 8), np.zeros(8)]])
    # rotate about the bottom center
    c, s = np.cos(theta), np.sin(theta)
    y = pts[:, 0] * c + pts[:, 1] * s
    z = -pts[:, 0] * s + pts[:, 1] * c
    return np.c_[y + yc, z + zb]


def box_dist(p, ylo, yhi, zlo, zhi):
    dy = np.maximum(np.maximum(ylo - p[:, 0], p[:, 0] - yhi), 0)
    dz = np.maximum(np.maximum(zlo - p[:, 1], p[:, 1] - zhi), 0)
    inside = (p[:, 0] > ylo) & (p[:, 0] < yhi) & (p[:, 1] > zlo) & (p[:, 1] < zhi)
    d = np.hypot(dy, dz)
    d[inside] = -np.minimum.reduce([p[inside, 0] - ylo, yhi - p[inside, 0],
                                    p[inside, 1] - zlo, zhi - p[inside, 1]])
    return d


def clearance(yc, zb, theta):
    p = outline(yc, zb, theta)
    d = np.minimum.reduce([
        box_dist(p, -1.0, Y_HANDLE, Z_H_LO, Z_H_HI),
        box_dist(p, Y_WALL_IN, 1.0, -1.0, Z_WALL_TOP),
        box_dist(p, -1.0, Y_CAB, -1.0, 2.0),
        box_dist(p, -1.0, 1.0, -1.0, Z_FLOOR),
    ])
    return d.min()


if __name__ == "__main__":
    print("LEVEL bowl: best yc per bottom height zb (clearance mm)")
    for zb in np.arange(1.00, 0.925, -0.005):
        ys = np.arange(-0.19, -0.08, 0.001)
        cl = [clearance(y, zb, 0) for y in ys]
        i = int(np.argmax(cl))
        print(f"  zb={zb:.3f} rim={zb+H:.3f}: yc={ys[i]:.3f} clearance={cl[i]*1000:.1f}")
    print("TILTED: best (yc, theta) per zb")
    for zb in np.arange(1.00, 0.925, -0.005):
        best = (-1, 0, 0)
        for th in np.radians(np.arange(-40, 41, 5)):
            for y in np.arange(-0.19, -0.08, 0.001):
                c = clearance(y, zb, th)
                if c > best[0]:
                    best = (c, y, th)
        print(f"  zb={zb:.3f}: yc={best[1]:.3f} theta={np.degrees(best[2]):.0f}deg clearance={best[0]*1000:.1f}")
