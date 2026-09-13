#!/usr/bin/env python3
"""Offline pixel -> world from a grab.py npz. Usage: px.py <prefix> u v [u v ...]"""
import sys
import numpy as np

d = np.load(sys.argv[1] + ".npz")
K, T, dep = d["K"], d["T"], d["depth"]
args = list(map(int, sys.argv[2:]))
for u, v in zip(args[::2], args[1::2]):
    z = float(dep[v, u])
    p = np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, 1.0])
    w = T @ p
    print(f"px({u},{v}) depth={z:.4f} -> world {w[0]:.4f} {w[1]:.4f} {w[2]:.4f}")
