#!/usr/bin/env python3
"""Segment above-table blobs in a top-down grab and report world stats.
Usage: python3 blobs.py <camera> <table_depth> [min_px]
"""
import json
import sys

import numpy as np
import cv2

cam = sys.argv[1]
table = float(sys.argv[2])
min_px = int(sys.argv[3]) if len(sys.argv) > 3 else 30
meta = json.load(open(f"{cam}_meta.json"))
depth = np.load(f"{cam}_depth.npy")
color = cv2.imread(f"{cam}.png")
K = np.array(meta["K"]).reshape(3, 3)
T = np.array(meta["T"])
fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

mask = ((depth < table - 0.01) & (depth > 0.5)).astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < min_px:
        continue
    ys, xs = np.where(lab == i)
    zs = depth[ys, xs]
    # world coords of all pixels
    P = np.stack([(xs - cx) * zs / fx, (ys - cy) * zs / fy, zs, np.ones_like(zs)])
    W = (T @ P)[:3]
    bgr = color[ys, xs].mean(0)
    x0, y0, w, h = stats[i, :4]
    print(f"blob {i}: px area={stats[i,4]} bbox=({x0},{y0},{w},{h}) "
          f"centroid px=({cents[i][0]:.0f},{cents[i][1]:.0f}) "
          f"world x[{W[0].min():.3f},{W[0].max():.3f}] y[{W[1].min():.3f},{W[1].max():.3f}] "
          f"ztop={W[2].max():.3f} zmin={W[2].min():.3f} mean bgr={bgr.round(0)}")
