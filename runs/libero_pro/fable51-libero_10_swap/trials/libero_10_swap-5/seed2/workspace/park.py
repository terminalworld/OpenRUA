import math, numpy as np, sys
from robot import Robot, yaw_down_quat
r = Robot("park")
x,y,z = map(float, sys.argv[1:4]); yaw = float(sys.argv[4]) if len(sys.argv)>4 else 0.0
q = yaw_down_quat(math.radians(yaw - 45.0))
for attempt in range(3):
    tcp = r.move_tcp([x,y,z], q, 3.0)
    if tcp is not None and np.linalg.norm(tcp-[x,y,z]) < 0.005: break
