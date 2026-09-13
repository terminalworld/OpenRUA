import numpy as np
from rob import *
r = Robot("s4")
Q = yaw_down_quat(0.0)
px, py = -0.0015, 0.3221          # right plate centre
tx, ty = px, py - 0.041           # TCP so that mug centre sits over plate centre
print("up  ", r.move_tcp((-0.0812, -0.197, 0.72), Q, 2.0), r.fingers())
print("mid ", r.move_tcp((-0.05, 0.05, 0.72), Q, 3.0), r.fingers())
print("over", r.move_tcp((tx, ty, 0.70), Q, 3.0), r.fingers())
print("down", r.move_tcp((tx, ty, 0.548), Q, 2.5), r.fingers())
print("f", np.round(r.force(),3))
print("open", r.gripper(0.04))
print("lift", r.move_tcp((tx, ty, 0.68), Q, 2.0))
