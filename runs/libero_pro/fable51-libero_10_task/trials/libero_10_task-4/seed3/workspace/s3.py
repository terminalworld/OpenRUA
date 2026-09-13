import numpy as np
from rob import *
r = Robot("s3")
Q = yaw_down_quat(0.0)
gx, gy = -0.0813, -0.197
print("f0", np.round(r.force(),3))
print("pre ", r.move_tcp((gx, gy, 0.60), Q, 2.0))
print("down", r.move_tcp((gx, gy, 0.519), Q, 2.0))
print("f1", np.round(r.force(),3))
print("close", r.gripper(0.0))
print("f2", np.round(r.force(),3))
print("lift", r.move_tcp((gx, gy, 0.65), Q, 2.0))
print("fingers after lift", r.fingers(), "f3", np.round(r.force(),3))
