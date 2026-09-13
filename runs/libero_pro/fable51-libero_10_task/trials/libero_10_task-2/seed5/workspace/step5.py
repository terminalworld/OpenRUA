import numpy as np, math
from rob import *
r = Robot()
H = np.array([-0.078, -0.05])
r.move_pose([H[0], H[1], 1.15], down_quat(math.pi/2), seconds=4.0, at_tcp=True)
print("q", np.round(r.arm_q(), 4))
