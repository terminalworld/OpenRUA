import numpy as np, math
from rob import *
r = Robot()
H = np.array([-0.078, -0.05])
Q = down_quat(math.pi/2)
r.move_pose([H[0], H[1], 1.10], Q, seconds=2.5, at_tcp=True)
print("gap", r.finger_gap())
w = r.wrench; print("wrench", w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2)))
