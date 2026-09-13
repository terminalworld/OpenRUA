import numpy as np
from rob import *
r = Robot()
r.gripper(0.04)
KNOB = np.array([-0.207, 0.204])
q = r.move_pose([KNOB[0], KNOB[1], 1.05], down_quat(0.0), seconds=4.0, at_tcp=True)
print("q now", np.round(r.arm_q(), 4))
