#!/usr/bin/env python3
"""Build a world point cloud from a saved npz and print a height map summary."""
import sys
import numpy as np


def cloud(cam):
    d = np.load(f"{cam}.npz")
    depth, K, T, color = d["depth"], d["K"], d["T"], d["color"]
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    z = depth
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    p = np.stack([x, y, z, np.ones_like(z)], -1).reshape(-1, 4)
    pw = (T @ p.T).T[:, :3].reshape(h, w, 3)
    return pw, color


if __name__ == "__main__":
    cam = sys.argv[1]
    pw, color = cloud(cam)
    Z = pw[..., 2]
    ok = np.isfinite(Z) & (pw[..., 0] > -0.7) & (pw[..., 0] < 1.0) & (np.abs(pw[..., 1]) < 1.0)
    print("z percentiles", np.percentile(Z[ok], [1, 5, 25, 50, 75, 95, 99]))
    # coarse height histogram
    hist, edges = np.histogram(Z[ok], bins=np.arange(0.5, 1.6, 0.02))
    for c, e in zip(hist, edges):
        if c > 50:
            print(f"{e:.2f} {c}")
