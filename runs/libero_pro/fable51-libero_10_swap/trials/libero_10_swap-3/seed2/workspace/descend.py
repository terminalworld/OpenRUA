#!/usr/bin/env python3
"""Step the held bowl down into the drawer, re-measuring with the wrist
camera after every step and correcting y. Aborts (and lifts) on stall.

Corridor while the rim passes the middle handle (z 1.005-1.035):
    yc in (-0.134, -0.1215)  -> aim -0.128
Once the rim is below 1.005, shift to yc=-0.145, xc=-0.10, bottom 0.93.
"""
import sys

import numpy as np

import ctl
import insert

r = ctl.Robot("descend")
Q = ctl.Q_DOWN_FINGERS_X
OFF = np.array([0.0499, 0.0021])       # bowl centre - TCP (x, y)
DZ = 0.0348                             # TCP z - bowl bottom z


def step(target_bowl_xy, bottom, seconds=3.0):
    tcp = np.array([target_bowl_xy[0] - OFF[0], target_bowl_xy[1] - OFF[1], bottom + DZ])
    q = r.ik_tcp(tcp, Q)
    if q is None:
        print("IK failed", file=sys.stderr)
        return False
    code, err = r.move_q(q, seconds)
    if code != 0 or err > 0.02:
        print(f"STALL code={code} err={err:.4f} -> lifting 3cm", file=sys.stderr)
        here = r.fk_tcp()[0]
        r.move_tcp(here + [0, 0, 0.03], Q, 2.0)
        return False
    return True


def check(tag):
    m = insert.measure()
    print(f"[{tag}]", end=" ")
    tcp, (bx, by), zb = insert.report(m, r)
    return bx, by, zb


bx, by, zb = check("start")
yc = -0.128
# phase 1: descend through the handle band, correcting y each step
for bottom in (1.02, 1.00, 0.985, 0.97, 0.955):
    # correct with measured bowl position (offset drift), keep target yc
    corr = np.array([bx, by]) - (r.fk_tcp()[0][:2] + OFF)
    tgt = np.array([bx, yc]) - corr
    if not step(tgt, bottom):
        sys.exit(1)
    bx, by, zb = check(f"bottom={bottom}")
    if not (-0.136 < by < -0.119):
        print("bowl y out of corridor, stopping", file=sys.stderr)
        sys.exit(1)
# phase 2: below the handle -> shift to final xy and set down
corr = np.array([bx, by]) - (r.fk_tcp()[0][:2] + OFF)
tgt = np.array([-0.10, -0.145]) - corr
if not step(tgt, 0.94, 3.0):
    sys.exit(1)
bx, by, zb = check("final xy")
if not step(r.fk_tcp()[0][:2] + OFF, 0.928, 2.0):   # straight down, no xy change
    sys.exit(1)
check("set down")
print("DONE descent; bowl still grasped", flush=True)
