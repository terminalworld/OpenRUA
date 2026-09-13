import numpy as np
from rob import *
r = Robot("s7")
Q = yaw_down_quat(0.0)
px, py = -0.0014, -0.3089         # left plate centre
tx, ty = px, py + 0.044           # TCP so mug centre sits over plate centre
print("mid ", r.move_tcp((-0.05, -0.05, 0.72), Q, 3.0), r.fingers())
print("over", r.move_tcp((tx, ty, 0.70), Q, 3.0), r.fingers())
f0 = r.force()
print("down1", r.move_tcp((tx, ty, 0.545), Q, 2.5), r.fingers(), "df", np.round(r.force()-f0,3))
print("down2", r.move_tcp((tx, ty, 0.530), Q, 1.5), r.fingers(), "df", np.round(r.force()-f0,3))
print("open", r.gripper(0.04))
print("lift", r.move_tcp((tx, ty, 0.68), Q, 2.0))
