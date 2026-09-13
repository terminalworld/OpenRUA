#!/usr/bin/env python3
"""Offline pixel -> world using saved arrays from locate.py.

Usage: python3 px.py <camera> u v [u v ...]
"""
import sys
import numpy as np

cam = sys.argv[1]
depth = np.load(f"{cam}_depth.npy")
meta = np.load(f"{cam}_meta.npy", allow_pickle=True).item()
K, T = meta["K"], meta["T"]
fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]


def to_world(u, v):
    z = float(depth[v, u])
    p = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
    return (T @ p)[:3], z


vals = list(map(int, sys.argv[2:]))
for u, v in zip(vals[::2], vals[1::2]):
    w, z = to_world(u, v)
    print(f"({u},{v}) depth={z:.3f} -> world {w[0]:.3f} {w[1]:.3f} {w[2]:.3f}")
