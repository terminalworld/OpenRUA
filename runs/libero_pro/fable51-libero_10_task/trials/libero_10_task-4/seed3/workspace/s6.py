import numpy as np
from rob import *
r = Robot("s6")
Q = yaw_down_quat(0.0)
gx, gy = -0.0315, 0.139
print("pre ", r.move_tcp((gx, gy, 0.60), Q, 2.0))
print("down", r.move_tcp((gx, gy, 0.512), Q, 2.0))
print("f1", np.round(r.force(),3))
print("close", r.gripper(0.0))
print("lift", r.move_tcp((gx, gy, 0.72), Q, 2.5))
print("fingers after lift", r.fingers(), "f", np.round(r.force(),3))
