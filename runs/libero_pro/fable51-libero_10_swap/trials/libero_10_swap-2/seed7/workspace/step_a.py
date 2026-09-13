import numpy as np
from lib import Robot
r = Robot("stepa")
r.gripper(0.04)
print("gap after open:", r.finger_gap())
q = r.move_pose([-0.21, 0.194, 1.15], (1,0,0,0), sec=4.0)
print("q:", np.round(q,4))
print("wrench:", r.wrench())
