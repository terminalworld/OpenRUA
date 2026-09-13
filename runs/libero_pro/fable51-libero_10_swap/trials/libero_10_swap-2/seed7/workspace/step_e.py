import numpy as np
from lib import Robot
r = Robot("stepe")
r.gripper(0.04)
p, quat, _ = r.fk_world(); print("hand at", np.round(p,4), np.round(quat,4))
r.move_pose(p + [0,0,0.12], quat, sec=2.5)
r.gripper(0.04)
