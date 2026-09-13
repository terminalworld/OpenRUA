import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
def wr():
    w = r.wrench; return w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2))
for z in [0.995, 0.975]:
    r.move_pose([0.045, 0.204, z], Q, seconds=2.0, at_tcp=True); print("gap", r.finger_gap(), "wrench", wr())
r.gripper(0.04)
r.move_pose([0.045, 0.204, 1.10], Q, seconds=2.5, at_tcp=True)
