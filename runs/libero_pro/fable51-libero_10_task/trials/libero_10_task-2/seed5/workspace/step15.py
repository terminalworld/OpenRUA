import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
def wr():
    w = r.wrench; return w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2))
r.gripper(0.0)
r.move_pose([0.080, 0.20, 1.10], Q, seconds=5.0, at_tcp=True)
r.move_pose([0.080, 0.20, 0.955], Q, seconds=3.0, at_tcp=True); print("wrench", wr())
r.move_pose([0.052, 0.20, 0.955], Q, seconds=3.0, at_tcp=True); print("wrench", wr())
r.move_pose([0.052, 0.20, 1.10], Q, seconds=2.5, at_tcp=True)
