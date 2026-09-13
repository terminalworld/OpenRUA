import numpy as np, sys
from lib import Robot
r = Robot("stepj")
dz = float(sys.argv[1])
p, quat, _ = r.fk_world()
r.move_pose(p + [0,0,dz], quat, sec=max(1.0, abs(dz)*8))
print("gap:", round(r.finger_gap(),4), "wrench:", np.round(r.wrench()[0],3))
