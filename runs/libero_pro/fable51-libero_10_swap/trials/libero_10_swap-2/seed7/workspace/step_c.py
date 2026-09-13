import numpy as np
from lib import Robot
r = Robot("stepc")
r.gripper(0.0)
r.spin(0.5)
print("gap:", round(r.finger_gap(),4), "wrench:", np.round(r.wrench()[0],3))
