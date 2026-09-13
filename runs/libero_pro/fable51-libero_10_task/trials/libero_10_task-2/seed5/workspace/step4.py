import numpy as np
from rob import *
r = Robot()
print("q", np.round(r.arm_q(), 4))
p, qt = r.tcp_pose()
r.move_pose([p[0], p[1], 1.08], qt, seconds=2.5, at_tcp=True)
