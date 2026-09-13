import numpy as np
from rob import *
r = Robot()
KNOB = np.array([-0.207, 0.204])
r.move_pose([KNOB[0], KNOB[1], 0.98], down_quat(0.0), seconds=2.5, at_tcp=True)
r.move_pose([KNOB[0], KNOB[1], 0.935], down_quat(0.0), seconds=2.0, at_tcp=True)
print("q now", np.round(r.arm_q(), 4))
w = r.wrench
print("wrench", w and (round(w.wrench.force.x,2), round(w.wrench.force.y,2), round(w.wrench.force.z,2)))
