#!/usr/bin/env python3
"""2D (top view) footprint check of hand + held mug against the microwave/door
for the insertion path.  Hand pose: TCP at p, hand z = a(yaw) = (-sin, cos, 0),
hand x = -world z, hand y = (-cos, -sin, 0).  Mug centre = TCP + 0.0785*a."""
import numpy as np

GRIP_TO_CENTRE = 0.0785
MUG_R = 0.05
# obstacles as axis-aligned boxes (xlo, xhi, ylo, yhi)
OBS = [
    (-0.322, -0.290, -0.590, -0.285),    # open door panel
    (-0.300, -0.260, -0.320, -0.290),    # front wall, -x of opening
    (-0.060, 0.055, -0.320, -0.290),     # front wall, +x of opening (incl. control panel)
    (-0.300, -0.266, -0.320, -0.110),    # left side wall
    (-0.057, 0.055, -0.320, -0.110),     # right side wall + panel
    (-0.300, 0.055, -0.144, -0.110),     # back wall
]


def frame(yaw):
    a = np.array([-np.sin(yaw), np.cos(yaw)])
    yh = np.array([-np.cos(yaw), -np.sin(yaw)])
    return a, yh


def footprint(tcp, yaw, n=9):
    """Return list of (points, label) for hand body, fingers, mug body, handle."""
    a, yh = frame(yaw)
    tcp = np.asarray(tcp[:2], float)
    pts = {}
    # hand body: s in +-0.103, t in [-0.1034, -0.0374]
    s = np.linspace(-0.103, 0.103, 11); t = np.linspace(-0.1034, -0.0374, 5)
    S, T = np.meshgrid(s, t)
    pts["hand"] = tcp + S.reshape(-1, 1) * yh + T.reshape(-1, 1) * a
    s = np.linspace(-0.05, 0.05, 6); t = np.linspace(-0.045, 0.012, 4)
    S, T = np.meshgrid(s, t)
    pts["fingers"] = tcp + S.reshape(-1, 1) * yh + T.reshape(-1, 1) * a
    c = tcp + GRIP_TO_CENTRE * a
    th = np.linspace(0, 2 * np.pi, 48)
    pts["mug"] = c + MUG_R * np.stack([np.cos(th), np.sin(th)], -1)
    pts["handle"] = tcp + np.linspace(-0.012, 0.04, 6).reshape(-1, 1) * a
    return pts, c


def collide(tcp, yaw, margin=0.0):
    pts, c = footprint(tcp, yaw)
    hits = []
    for name, P in pts.items():
        for (xlo, xhi, ylo, yhi) in OBS:
            m = (P[:, 0] > xlo - margin) & (P[:, 0] < xhi + margin) & (P[:, 1] > ylo - margin) & (P[:, 1] < yhi + margin)
            if m.any():
                hits.append((name, (xlo, xhi, ylo, yhi)))
                break
    return hits


def tcp_for_mug(mug_c, yaw):
    a, _ = frame(yaw)
    return np.asarray(mug_c, float) - GRIP_TO_CENTRE * a


def check_path(waypoints, n=40, margin=0.0):
    """waypoints: list of (mug_x, mug_y, yaw_deg). Linear interpolation. Returns list of problems."""
    W = np.array(waypoints, float)
    probs = []
    for i in range(len(W) - 1):
        for u in np.linspace(0, 1, n, endpoint=(i == len(W) - 2)):
            w = W[i] * (1 - u) + W[i + 1] * u
            yaw = np.deg2rad(w[2])
            tcp = tcp_for_mug(w[:2], yaw)
            h = collide(tcp, yaw, margin)
            if h:
                probs.append((np.round(w, 3).tolist(), np.round(tcp, 3).tolist(), h))
    return probs


if __name__ == "__main__":
    import sys
    for yaw in (0, -15, -25, -35):
        for my in (-0.40, -0.37, -0.34, -0.31, -0.28, -0.25, -0.22):
            for mx in (-0.13, -0.15, -0.17, -0.19, -0.21):
                tcp = tcp_for_mug((mx, my), np.deg2rad(yaw))
                h = collide(tcp, np.deg2rad(yaw))
                print(f"yaw {yaw:4d} mug ({mx:.2f},{my:.2f}) tcp ({tcp[0]:.3f},{tcp[1]:.3f}) ->", "ok" if not h else [x[0] for x in h])
