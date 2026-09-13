import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
r.move_pose([0.012, -0.23, 1.12], Q, seconds=2.5, at_tcp=True)
print("gap", r.finger_gap())
