import numpy as np
from rob import *
r = Rob()
x, y = 0.0065, -0.2285
print("descend"); r.move_tcp(x, y, 0.48, sec=3)
print("final"); r.move_tcp(x, y, 0.440, sec=3)
f = r.gripper(0.0)
print("lift"); r.move_tcp(x, y, 0.60, sec=3)
print("fingers after lift", r.fingers())
r.shutdown()
