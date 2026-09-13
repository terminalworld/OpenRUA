import numpy as np, sys
from geom import R_PICK, Rx
from rob import Robot, log
R = Rx(np.radians(25)) @ R_PICK
def hand(t): return np.array(t) - 0.1034 * R[:, 2]
hx, hy = -0.1995, -0.267
r = Robot("setdown")
assert r.move_pose(hand((hx, hy, 1.06)), R, 3.0)
log("fingers", r.fingers())
assert r.move_pose(hand((hx, hy, 1.015)), R, 2.0)
log("fingers", r.fingers())
assert r.move_pose(hand((hx, hy, 1.000)), R, 1.5)
log("fingers", r.fingers())
r.gripper(0.04)
assert r.move_pose(hand((hx, hy, 1.20)), R, 3.0)
log("SETDOWN DONE")
