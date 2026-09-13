#!/usr/bin/env python3
"""Pull the bottom drawer out further using the -y finger behind the front panel."""
import sys
import numpy as np
from rlib import *

r = Robot("step1")
print("start:"); r.report()
R = top_down_R(np.pi / 2)            # fingers along world y
x_c, y_h = -0.11, -0.071
z_hi, z_lo = 1.10, 0.965

if r.finger() < 0.035:
    r.gripper(0.04)

# a. pre-pose above the drawer
q_pre = r.ik([x_c, y_h, z_hi], R)
p, Rk = r.tcp(q_pre); print("pre tcp", np.round(p, 4), "Z", np.round(Rk[:, 2], 3), "Y", np.round(Rk[:, 1], 3))
r.move_q(q_pre, 4.0)
r.report()

# b. descend
r.move_cart([([x_c, y_h, z_lo], R)], 2.5)
r.report()
print("wrench", np.round(r.wrench(), 2))

# c. pull +y
r.move_cart([([x_c, y_h + 0.07, z_lo], R)], 4.0)
r.report()
print("wrench", np.round(r.wrench(), 2))

# e. lift
r.move_cart([([x_c, y_h + 0.07, z_hi], R)], 2.5)
r.report()
