import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
gap = r.gripper(0.0)
r.move_pose([0.012, -0.23, 0.99], Q, seconds=2.5, at_tcp=True)
print("gap after lift", r.finger_gap())
w = r.wrench; print("wrench", w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2)))
