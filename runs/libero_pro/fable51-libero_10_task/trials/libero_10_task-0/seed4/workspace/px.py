#!/usr/bin/env python3
"""Offline pixel -> world using files from cam_capture.py.

Usage: python3 px.py <camera> u v [u v ...]
"""
import sys

import numpy as np

cam = sys.argv[1]
depth = np.load(f"{cam}_depth.npy")
m = np.load(f"{cam}_meta.npz")
K, T = m["K"], m["T"]
vals = list(map(int, sys.argv[2:]))
for u, v in zip(vals[::2], vals[1::2]):
    z = depth[v, u]
    p = np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, 1.0])
    w = T @ p
    print(f"({u},{v}) depth={z:.3f} -> world {w[0]:.4f} {w[1]:.4f} {w[2]:.4f}")
