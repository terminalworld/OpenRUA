import numpy as np
from rob import *
r = Robot("s5")
Q = yaw_down_quat(0.0)
print("hover", r.move_tcp((-0.035, 0.14, 0.66), Q, 3.0), r.fingers())
