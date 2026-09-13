import numpy as np, math
from rob import *
r = Robot()
H = np.array([-0.078, -0.05])
Q = down_quat(math.pi/2)
r.move_pose([H[0], H[1], 1.06], Q, seconds=2.5, at_tcp=True)
r.move_pose([H[0], H[1], 1.003], Q, seconds=2.0, at_tcp=True)
w = r.wrench; print("wrench", w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2)))
gap = r.gripper(0.0)
print("finger gap after close:", gap)
