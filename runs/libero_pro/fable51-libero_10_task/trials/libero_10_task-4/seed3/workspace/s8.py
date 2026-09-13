import numpy as np
from rob import *
r = Robot("s8")
home = [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]
print("home", r.move_joints(home, 3.0))
print("tcp", np.round(r.tcp_world()[0],4))
