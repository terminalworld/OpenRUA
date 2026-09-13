import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
def wr():
    w = r.wrench; return w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2))
r.move_pose([0.012, -0.23, 1.25], Q, seconds=2.5, at_tcp=True); print("gap", r.finger_gap())
r.move_pose([0.045, 0.204, 1.25], Q, seconds=5.0, at_tcp=True); print("gap", r.finger_gap(), "wrench", wr())
r.move_pose([0.045, 0.204, 1.02], Q, seconds=3.0, at_tcp=True); print("gap", r.finger_gap(), "wrench", wr())
