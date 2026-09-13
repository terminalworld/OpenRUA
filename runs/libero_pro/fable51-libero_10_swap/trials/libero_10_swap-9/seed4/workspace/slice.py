#!/usr/bin/env python3
"""slice.py <cam> x|y c0 [c1 ...] : print z-vs-(other axis) occupancy slices of <cam>_world.npy at coordinate(s) c (half-width 5mm)"""
import sys, numpy as np
cam, ax = sys.argv[1], sys.argv[2]
P = np.load(f'{cam}_world.npy').reshape(-1, 3); P = P[np.isfinite(P).all(1)]
i = 0 if ax == 'x' else 1; j = 1 - i
lo, hi = (-0.52, -0.28) if j == 1 else (-0.30, 0.04)
for c in map(float, sys.argv[3:]):
    m = (np.abs(P[:, i] - c) < 0.005) & (P[:, 2] > 0.903) & (P[:, 2] < 1.06) & (P[:, j] > lo) & (P[:, j] < hi)
    Q = P[m]; cs = np.arange(lo, hi, 0.005)
    print(f"=== {ax}={c:.3f}   {'y' if j else 'x'}:", ' '.join(f"{v*100:4.0f}" for v in cs))
    for z0 in np.arange(1.03, 0.90, -0.01):
        row = ''
        for c0 in cs:
            t = Q[(Q[:, j] > c0) & (Q[:, j] < c0 + 0.005) & (Q[:, 2] > z0) & (Q[:, 2] < z0 + 0.01)]
            row += f"{len(t):4d}" if len(t) else '   .'
        print(f"z {z0:.2f}", row)
