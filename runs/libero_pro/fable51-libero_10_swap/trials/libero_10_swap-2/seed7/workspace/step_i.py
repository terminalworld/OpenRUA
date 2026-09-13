import numpy as np
from lib import Robot
r = Robot("stepi")
quat = (0.8315, 0.5556, 0, 0)
r.move_pose([-0.0294, -0.237, 1.124], quat, sec=1.0)
print("wrench:", np.round(r.wrench()[0],3))
r.gripper(0.0)
r.spin(0.3)
print("gap:", round(r.finger_gap(),4), "wrench:", np.round(r.wrench()[0],3))
