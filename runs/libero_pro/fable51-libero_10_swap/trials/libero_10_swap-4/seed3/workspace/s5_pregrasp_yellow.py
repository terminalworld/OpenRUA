import numpy as np
from rob import Robot, topdown_quat
r = Robot("s5")
Q = topdown_quat(0.0)
C = np.array([-0.229, 0.030]); R_WALL = 0.040
pinch = [C[0], C[1] + R_WALL]
print("pinch", pinch)
if r.move_tcp([pinch[0], pinch[1], 0.70], Q, seconds=4.0) is None: raise SystemExit("fail")
print("joints", np.round(r.joints(), 4))
