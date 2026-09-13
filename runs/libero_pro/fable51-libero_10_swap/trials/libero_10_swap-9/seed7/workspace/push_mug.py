#!/usr/bin/env python3
"""Push the yellow mug +y with the closed, vertical gripper (fingers along x)."""
import sys
import numpy as np
from ctl import Robot, quat_down, log

r = Robot("push")
Q = quat_down(90)          # vertical, finger axis along world x
MX = 0.015                 # mug/handle x
Y0, Y1 = -0.07, 0.08       # tip start / end y
ZP = 0.925

def ik_or_die(p, seed=None):
    s = r.ik(p, Q, seed=seed, at_tcp=True)
    if s is None:
        sys.exit(f"IK failed at {p}")
    return s

seed = r.joints()
approach = []
for p in [(-0.15, -0.45, 1.30), (-0.05, -0.25, 1.22), (MX, Y0, 1.12)]:
    s = ik_or_die(p, seed)
    log(f"wp {p} -> {np.round(s,2)} jump={np.abs(np.array(s)-np.array(seed)).max():.2f}")
    approach.append(s); seed = s
down = ik_or_die((MX, Y0, ZP), seed)
push = []
seed = down
for y in np.linspace(Y0, Y1, 6)[1:]:
    s = ik_or_die((MX, y, ZP), seed); push.append(s); seed = s
up = ik_or_die((MX, Y1, 1.10), seed)
if "dry" in sys.argv:
    sys.exit(0)
log("== close gripper"); r.gripper(0.0)
log("== approach"); r.move_joints(approach, [5.0, 9.0, 13.0])
log("hand", np.round(r.hand_pose()[0], 3))
log("== descend"); r.move_joints([down], [4.0])
log("== push"); r.move_joints(push, [2.0 + 1.5 * i for i in range(len(push))], hold=3.0)
log("hand", np.round(r.hand_pose()[0], 3))
log("== up"); r.move_joints([up], [3.0])
