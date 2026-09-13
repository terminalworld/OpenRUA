#!/usr/bin/env python3
"""Rim-grasp pick and place of a mug onto a plate.

Usage: python3 pick_place.py <phase> ...
  pick  mx my rim_z side(+1|-1)   grasp the mug wall on the +y/-y side
  place px py                     put the held mug centred on the plate
Coordinates are world; hand always points straight down (Q_DOWN).
"""
import sys
import time

import numpy as np

from robot import Q_DOWN, Robot

WALL_R = 0.036      # mug wall radius (centre of the wall)
Z_TRAVEL = 0.68     # safe travel height for the TCP
GRASP_BELOW_RIM = 0.03
MUG_H = 0.11        # mug height (bottom -> rim)
PLATE_TOP = 0.447


def goto(r, pos, secs=3.0, tol=0.01):
    """IK + trajectory with one retry; returns final TCP."""
    for attempt in range(3):
        q = r.ik_world(pos, Q_DOWN)
        if q is None:
            raise SystemExit(f"IK failed for {pos}")
        code, err = r.move_joints(q, secs)
        tcp = r.tcp_world()
        d = np.linalg.norm(tcp - np.array(pos))
        print(f"  goto {np.array(pos).round(3)} -> tcp {tcp.round(4)} (|d|={d:.4f}) code={code}", flush=True)
        if code == 0 and d < tol:
            return tcp
        print("  retrying goal", flush=True)
    return tcp


def pick(r, mx, my, rim_z, side):
    gy = my + side * WALL_R
    print(f"PICK mug at ({mx},{my}) rim {rim_z}, wall grasp at y={gy:.3f}", flush=True)
    r.gripper(0.04)
    goto(r, (mx, gy, Z_TRAVEL), 4.0)
    goto(r, (mx, gy, rim_z + 0.02), 3.0)        # just above the rim
    goto(r, (mx, gy, rim_z - GRASP_BELOW_RIM), 2.0)  # fingers straddle wall
    f = r.gripper(0.0)
    gap = f[0] - f[1] if f[1] < 0 else f[0] + f[1]
    print(f"  finger gap after close: {gap:.4f}", flush=True)
    goto(r, (mx, gy, Z_TRAVEL), 3.0)
    f = r.fingers()
    print(f"  fingers after lift: {f}", flush=True)
    return gy


def place(r, px, py, side):
    gy = py + side * WALL_R
    print(f"PLACE on plate ({px},{py}), tcp y={gy:.3f}", flush=True)
    goto(r, (px, gy, Z_TRAVEL), 4.0)
    z_set = PLATE_TOP + (MUG_H - GRASP_BELOW_RIM) + 0.008
    goto(r, (px, gy, z_set + 0.04), 3.0)
    goto(r, (px, gy, z_set), 2.0)
    r.gripper(0.04)
    goto(r, (px, gy, Z_TRAVEL), 3.0)


if __name__ == "__main__":
    r = Robot()
    a = sys.argv[1:]
    t0 = time.time()
    if a[0] == "pick":
        pick(r, float(a[1]), float(a[2]), float(a[3]), float(a[4]))
    elif a[0] == "place":
        place(r, float(a[1]), float(a[2]), float(a[3]))
    elif a[0] == "goto":
        goto(r, tuple(map(float, a[1:4])))
    print(f"done in {time.time()-t0:.0f}s wall", flush=True)
