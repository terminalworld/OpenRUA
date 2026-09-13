import sys, numpy as np
sys.path.insert(0, "/workspace")
from lib import Robot
r = Robot()
w0 = r.wrench()
print("fingers before", np.round(r.fingers(), 4))
res = r.gripper(float(sys.argv[1]))
print("reached_goal, stalled, fingers:", res)
r.spin(0.5)
print("fingers after settle", np.round(r.fingers(), 4))
print("wrench before", np.round(w0, 2)); print("wrench after ", np.round(r.wrench(), 2))
r.close()
