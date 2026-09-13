"""Cheap collision check: arm capsules + hand box vs world AABBs."""
import math
import numpy as np
from panda import DH, _tf, rotz, BASE_OFF, fk_tcp

TABLE_Z = 0.90
# name: (min xyz, max xyz) in world
OBST = {
    "shelf": ([-0.245, -0.38, 0.90], [0.04, -0.17, 1.25]),
    "cabinet": ([-0.14, 0.228, 0.90], [0.145, 0.44, 1.14]),
    "tophandle": ([-0.05, 0.185, 1.01], [0.06, 0.23, 1.11]),
    "drawer": ([-0.115, 0.035, 0.90], [0.135, 0.23, 1.0]),
    "bottle": ([-0.08, 0.09, 0.90], [-0.01, 0.16, 1.09]),
    "bowl": ([0.13, -0.09, 0.90], [0.26, 0.03, 0.955]),
}
LINK_R = 0.055


def frames(q):
    """World poses of link frames 1..7, flange, hand."""
    T = np.eye(4)
    T[:3, 3] = BASE_OFF
    out = [T.copy()]
    for (a, d, al), th in zip(DH, q):
        T = T @ _tf(a, d, al, th)
        out.append(T.copy())
    T = T @ _tf(0, 0.107, 0, 0)
    out.append(T.copy())  # flange
    T = T @ rotz(-math.pi / 4)
    out.append(T.copy())  # hand
    return out


def pt_box_dist(p, lo, hi):
    d = np.maximum(np.maximum(lo - p, 0), p - hi)
    return np.linalg.norm(d, axis=-1)


def hand_points(Th):
    """Sample points of the hand box + fingers in world."""
    pts = []
    for x in (-0.03, 0.03):
        for y in np.linspace(-0.10, 0.10, 9):
            for z in (-0.04, 0.0, 0.035, 0.07):
                pts.append([x, y, z, 1])
    for x in (-0.012, 0.012):
        for y in (-0.05, -0.025, 0, 0.025, 0.05):
            for z in (0.09, 0.115):
                pts.append([x, y, z, 1])
    P = (Th @ np.array(pts).T).T[:, :3]
    return P


def arm_points(q, n=6):
    fr = frames(q)
    pts = []
    # capsule segments between consecutive frame origins (skip base->1 which is the pedestal column)
    origins = [f[:3, 3] for f in fr]
    for i in range(1, len(origins) - 2):  # up to flange
        a, b = origins[i], origins[i + 1]
        for s in np.linspace(0, 1, n):
            pts.append(a * (1 - s) + b * s)
    return np.array(pts), fr[-1]


def check(q, ignore=(), verbose=False, extra=None):
    """Return (min clearance, worst obstacle name). Negative = collision."""
    obst = dict(OBST)
    if extra:
        obst.update(extra)
    P, Th = arm_points(q)
    H = hand_points(Th)
    worst = (1e9, None)
    for name, (lo, hi) in obst.items():
        if name in ignore:
            continue
        lo, hi = np.array(lo), np.array(hi)
        d1 = pt_box_dist(P, lo, hi).min() - LINK_R
        d2 = pt_box_dist(H, lo, hi).min()
        d = min(d1, d2)
        if verbose:
            print(f"  {name}: arm {d1:+.3f} hand {d2:+.3f}")
        if d < worst[0]:
            worst = (d, name)
    # table
    dt = min(P[:, 2].min() - LINK_R, H[:, 2].min()) - TABLE_Z
    if verbose:
        print(f"  table: {dt:+.3f}")
    if dt < worst[0]:
        worst = (dt, "table")
    return worst


def check_path(qs, ignore=(), steps=10, extra=None):
    worst = (1e9, None, -1)
    for i in range(len(qs) - 1):
        for s in np.linspace(0, 1, steps):
            q = np.array(qs[i]) * (1 - s) + np.array(qs[i + 1]) * s
            d, n = check(q, ignore, extra=extra)
            if d < worst[0]:
                worst = (d, n, i + s)
    return worst
