import numpy as np
from rob import Robot, topdown_quat
r = Robot("s1")
Q = topdown_quat(0.0)           # fingers close along world y
WHITE_C = np.array([-0.130, -0.189]); R_WALL = 0.042; RIM_Z = 0.549
pinch = np.array([WHITE_C[0], WHITE_C[1] - R_WALL])
print("pinch point", pinch)
r.gripper(0.04)
print("pre-grasp above pinch, z=0.65")
r.move_tcp([pinch[0], pinch[1], 0.65], Q, seconds=4.0)
print("joints", np.round(r.joints(), 4))
