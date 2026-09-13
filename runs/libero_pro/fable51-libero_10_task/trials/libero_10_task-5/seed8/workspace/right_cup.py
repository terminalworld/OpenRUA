#!/usr/bin/env python3
"""The cup is held by its rim and has swung ~80 deg about the finger axis
(base toward +x). The pinch acts as a friction hinge stiff enough to hold
the cup's weight, so rotate the whole hand about world Y to bring the cup
back upright, then set it down on the open table."""
import math
import sys
import numpy as np
from rob import *

taper = math.atan2(0.0125, 0.105)
R_place = hand_R(0.0, [1, 0, 0], taper)
P = [-0.30, -0.05, 1.08]
r = Robot()
step = sys.argv[1]
if step == "move":
    print(r.move_tcp([-0.15, -0.05, 1.08], R_place, 3.0))
    print(r.move_tcp(P, R_place, 4.0))
    print("gap %.4f" % r.finger_gap(), "F", np.round(r.wrench()[0], 2))
elif step == "rotate":
    q0 = r.arm_q()
    for deg in (25, 50, 65, 75):
        Rn = rot_y(math.radians(deg)) @ R_place
        ok, info = r.move_tcp(P, Rn, 4.0, seed=q0)
        q0 = r.arm_q()
        print(deg, info, "gap %.4f" % r.finger_gap(), "F", np.round(r.wrench()[0], 2))
        if not ok:
            break
    print("hand z", np.round(quat_to_R(r.hand_pose()[1])[:, 2], 3))
elif step == "down":
    deg = float(sys.argv[2])
    Rn = rot_y(math.radians(deg)) @ R_place
    for z in (1.0, 0.975, 0.96):
        ok, info = r.move_tcp([P[0], P[1], z], Rn, 2.5)
        F = r.wrench()[0]
        print(z, info, "F", np.round(F, 2))
        if abs(F[2] + 5.3) > 2.0:
            break
r.close()
