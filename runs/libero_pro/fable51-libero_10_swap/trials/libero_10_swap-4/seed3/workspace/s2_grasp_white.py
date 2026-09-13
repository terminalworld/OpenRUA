import numpy as np
from rob import Robot, topdown_quat
r = Robot("s2")
Q = topdown_quat(0.0)
C = np.array([-0.127, -0.188]); RIM_Z = 0.549; R_WALL = 0.042
pinch = [C[0], C[1] - R_WALL]
print("descend to just above rim")
if r.move_tcp([pinch[0], pinch[1], RIM_Z + 0.035], Q, seconds=3.0) is None: raise SystemExit("fail1")
print("descend into pinch depth")
if r.move_tcp([pinch[0], pinch[1], RIM_Z - 0.025], Q, seconds=2.5) is None: raise SystemExit("fail2")
print("wrench before close", r.read_wrench())
f = r.gripper(0.0)
print("wrench after close", r.read_wrench())
print("lift")
r.move_tcp([pinch[0], pinch[1], 0.70], Q, seconds=3.0)
print("fingers after lift", r.fingers(), "wrench", r.read_wrench())
