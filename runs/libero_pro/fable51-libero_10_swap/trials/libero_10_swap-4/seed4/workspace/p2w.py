#!/usr/bin/env python3
"""Pixel(s) -> world using saved grab.py outputs.
Usage: python3 p2w.py <camera> u v [u v ...]
"""
import json
import sys

import numpy as np

cam = sys.argv[1]
meta = json.load(open(f"{cam}_meta.json"))
depth = np.load(f"{cam}_depth.npy")
K = np.array(meta["K"]).reshape(3, 3)
T = np.array(meta["T"])
fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]


def p2w(u, v):
    z = float(depth[v, u])
    p = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
    return (T @ p)[:3]


vals = list(map(int, sys.argv[2:]))
for u, v in zip(vals[::2], vals[1::2]):
    w = p2w(u, v)
    print(f"({u},{v}) depth={depth[v,u]:.3f} -> world {w[0]:.3f} {w[1]:.3f} {w[2]:.3f}")
