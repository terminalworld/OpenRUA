import numpy as np
from rob import *
r = Robot("s1")
print("open:", r.gripper(0.04))
Q = yaw_down_quat(0.0)
code, err, pos = r.move_tcp((-0.079, -0.196, 0.66), Q, 3.0)
print("hover: code", code, "jerr", round(err,4), "tcp", np.round(pos,4))
print("force", np.round(r.force(),3))
