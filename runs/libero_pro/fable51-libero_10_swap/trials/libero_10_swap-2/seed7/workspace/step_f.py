import numpy as np, sys
from lib import Robot
r = Robot("stepf")
x, y, z = map(float, sys.argv[1:4]); quat = tuple(map(float, sys.argv[4:8])) if len(sys.argv) >= 8 else (0.7071, 0.7071, 0, 0)
sec = float(sys.argv[8]) if len(sys.argv) > 8 else 4.0
q = r.move_pose([x, y, z], quat, sec=sec)
print("q:", np.round(q,4) if q is not None else None, "gap:", round(r.finger_gap(),4))
