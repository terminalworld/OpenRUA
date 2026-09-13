import numpy as np
from rob import *
r = Robot("s2")
j = r.joints()
print(r.move_joints(j, 0.5))
print("tcp", np.round(r.tcp_world()[0],4))
