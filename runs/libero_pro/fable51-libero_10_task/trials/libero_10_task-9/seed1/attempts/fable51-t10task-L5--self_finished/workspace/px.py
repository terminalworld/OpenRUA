#!/usr/bin/env python3
"""Offline pixel -> world from a cam_cache.py npz.

Usage: python3 px.py <cam> u v [u v ...]
       python3 px.py <cam> --box u0 v0 u1 v1   (stats of world pts in box)
"""
import sys

import numpy as np


def load(cam):
    d = np.load(f"{cam}_cache.npz")
    return d["depth"], d["K"], d["T"]


def to_world(depth, K, T, u, v):
    z = depth[v, u]
    if not np.isfinite(z) or z <= 0:
        return None
    p = np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, 1.0])
    return (T @ p)[:3]


def main():
    cam = sys.argv[1]
    depth, K, T = load(cam)
    if sys.argv[2] == "--box":
        u0, v0, u1, v1 = map(int, sys.argv[3:7])
        pts = []
        for v in range(v0, v1 + 1):
            for u in range(u0, u1 + 1):
                p = to_world(depth, K, T, u, v)
                if p is not None:
                    pts.append(p)
        pts = np.array(pts)
        print("n=", len(pts))
        print("min ", pts.min(0))
        print("max ", pts.max(0))
        print("mean", pts.mean(0))
        print("median", np.median(pts, 0))
        return
    args = list(map(int, sys.argv[2:]))
    for u, v in zip(args[::2], args[1::2]):
        p = to_world(depth, K, T, u, v)
        print(f"({u},{v}) depth={depth[v,u]:.4f} ->", None if p is None else np.round(p, 4))


if __name__ == "__main__":
    main()
