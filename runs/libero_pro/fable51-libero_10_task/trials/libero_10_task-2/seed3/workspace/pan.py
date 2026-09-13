#!/usr/bin/env python3
"""Release knob, then grasp the pan handle and lift. Stage 1 of the place."""
import numpy as np
from robot import Robot, quat_down

r = Robot()
tcp, _ = r.fk()
print("start tcp", np.round(tcp, 4))
r.open()
# straight up off the knob
r.move_tcp([tcp[0], tcp[1], 1.05], quat_down(-45), 2.5)

# pan handle: runs along Y at x=-0.076, grasp mid-handle; fingers close along X
QH = quat_down(45)
GX, GY = -0.076, -0.06
print("-> above handle")
r.move_tcp([GX, GY, 1.05], QH, 4.0)
print("-> descend")
r.move_tcp([GX, GY, 0.930], QH, 2.5)
f = r.close()
print("finger after close", f)
print("-> lift")
r.move_tcp([GX, GY, 1.20], QH, 3.0)
print("finger after lift", r.finger())
print("DONE")
