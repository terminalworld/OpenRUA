import numpy as np, math
from rob import *
r = Robot()
Q = down_quat(math.pi/2)
r.move_pose([0.045, 0.204, 1.25], Q, seconds=2.5, at_tcp=True)
home = [0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854]
r.move_q(home, seconds=5.0)
print("q", np.round(r.arm_q(), 3))
