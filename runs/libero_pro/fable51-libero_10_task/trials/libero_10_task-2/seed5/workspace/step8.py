import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
r.gripper(0.04)
r.move_pose([-0.003, -0.22, 1.05], Q, seconds=4.0, at_tcp=True)
