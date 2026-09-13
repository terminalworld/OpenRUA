#!/usr/bin/env python3
"""Pick a moka pot by its handle plate (handle pointing -y).
Usage: python3 -u pick.py <hx> <hy>   (world x,y of the handle outer-arm grasp point)
Fingers close along world x; hand leaned 25 deg about x so palm sits outboard of the pot.
"""
import sys
import numpy as np
from geom import R_PICK, Rx
from rob import Robot, log

hx, hy = float(sys.argv[1]), float(sys.argv[2])
LEAN = np.radians(25)
R = Rx(LEAN) @ R_PICK                      # hand z -> (0, sin, -cos): down and toward +y (pot body)
Z_TIPS_GRASP = 0.999
Z_TIPS_HIGH = 1.20


def hand(tips):
    return np.array(tips) - 0.1034 * R[:, 2]


r = Robot("pick")
log("start joints", np.round(r.arm_q(), 3), "fingers", r.fingers())
r.gripper(0.04)

# 1. pre-grasp high above the handle
assert r.move_pose(hand((hx, hy, Z_TIPS_HIGH)), R, 4.0), "pre-grasp move failed"
# 2. descend in two legs
assert r.move_pose(hand((hx, hy, 1.06)), R, 2.5), "descend-1 failed"
assert r.move_pose(hand((hx, hy, Z_TIPS_GRASP)), R, 2.0), "descend-2 failed"
# 3. close
f = r.gripper(0.0)
gap = f[0] - f[1]
log(f"finger gap after close = {gap:.4f} m")
if gap < 0.003:
    log("GRASP FAILED: fingers closed on air")
    sys.exit(2)
# 4. lift straight up
assert r.move_pose(hand((hx, hy, Z_TIPS_HIGH)), R, 3.0), "lift failed"
f = r.fingers()
log(f"after lift finger gap = {f[0]-f[1]:.4f}")
log("PICK DONE")
