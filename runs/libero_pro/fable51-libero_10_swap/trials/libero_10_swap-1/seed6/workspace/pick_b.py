import numpy as np
from rob import *
r = Rob()
x, y = -0.111, 0.041
print("descend"); r.move_tcp(x, y, 0.50, sec=3)
print("final"); r.move_tcp(x, y, 0.442, sec=3)
f = r.gripper(0.0)
for i in range(3): print("fingers", r.fingers())
print("lift"); r.move_tcp(x, y, 0.60, sec=3)
print("fingers after lift", r.fingers())
r.shutdown()
