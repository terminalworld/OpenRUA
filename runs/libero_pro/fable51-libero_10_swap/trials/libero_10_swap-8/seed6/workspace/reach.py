#!/usr/bin/env python3
"""Reach survey (no motion): for several hand orientations, find IK feasibility of
fingertip positions along x at given y,z. Uses a few seeds."""
import numpy as np
from geom import R_PICK, R_PLACE, Ry, Rz, hand_from_tips
from rob import Robot, log

r = Robot("reach")
seeds = [r.arm_q(),
         np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]),
         np.array([0, 0.3, 0, -1.6, 0, 1.9, 0.785]),
         np.array([0, 0.6, 0, -1.2, 0, 1.8, 0.785])]


def feasible(pos, R):
    for s in seeds:
        for Rc in (R, R @ np.diag([-1.0, -1.0, 1.0])):
            q = r.ik(pos, Rc, s)
            if q is not None:
                return q
    return None


R_HORIZ = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]], float)  # hand z -> +x, fingers close along y
cases = {
    "vertical R_PLACE tips z1.044": (R_PLACE, 1.044),
    "vertical R_PICK tips z1.044": (R_PICK, 1.044),
    "lean25 tips z1.044": (Ry(np.radians(-25)) @ R_PLACE, 1.044),
    "lean45 tips z1.044": (Ry(np.radians(-45)) @ R_PLACE, 1.044),
    "vertical R_PLACE tips z0.95": (R_PLACE, 0.95),
    "horiz +x, hand z0.963": (R_HORIZ, None),
    "horiz +x, hand z1.00": (R_HORIZ, None),
}
import sys
sys.stdout = open("/dev/stdout", "w")
for name, (R, ztip) in cases.items():
    ok = []
    for x in np.arange(0.0, 0.26, 0.02):
        for y in (0.0, 0.08, -0.02):
            if ztip is None:
                pos = np.array([x, y, 0.963 if "0.963" in name else 1.00])
            else:
                pos = hand_from_tips((x, y, ztip), R)
            q = feasible(pos, R)
            ok.append((round(x, 2), y, q is not None))
    good = [(x, y) for x, y, f in ok if f]
    print(name, "feasible x per y:", {y: [x for x, yy, f in ok if f and yy == y] for y in (0.0, 0.08, -0.02)}, flush=True)
