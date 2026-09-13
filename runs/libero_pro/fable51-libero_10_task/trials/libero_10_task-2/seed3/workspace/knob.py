#!/usr/bin/env python3
"""Turn the stove knob: grasp the tab from above, rotate the wrist.
Usage: knob.py <delta_j7_rad>   (negative = CCW viewed from above)
"""
import sys
import numpy as np
from robot import Robot, quat_down

KNOB = np.array([-0.207, 0.191])
Z_ABOVE, Z_GRASP = 1.02, 0.932
# fingers close along world Y (tab lies along X): link8 yaw = -45 deg
Q = quat_down(-45)

delta = float(sys.argv[1])
r = Robot()
print("finger before", r.finger())
r.open()
print("-> above knob")
r.move_tcp([*KNOB, Z_ABOVE], Q, 4.0)
print("-> descend")
q = r.move_tcp([*KNOB, Z_GRASP], Q, 2.5)
print("j7 at grasp", q[6])
f = r.close()
print("finger after close", f)
q2 = list(r.arm_q())
q2[6] += delta
print("-> rotate j7 by", delta, "to", q2[6])
r.move_q(q2, 3.0)
tcp, _ = r.fk()
print("tcp after rotate", np.round(tcp, 4), "finger", r.finger())
print("DONE")
