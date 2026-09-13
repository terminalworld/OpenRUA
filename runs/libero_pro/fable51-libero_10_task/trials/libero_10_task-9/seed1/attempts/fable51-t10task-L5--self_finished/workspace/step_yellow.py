import numpy as np
from rob import *
r = Rob('yellow')
R_V = hand_R([0, 0, -1], [1, 0, 0])   # vertical, fingers along x
cx, cy, rimz, R = 0.0115, -0.0041, 1.004, 0.0477
grasp = np.array([cx + R, cy, rimz - 0.024])
print('fingers now', r.finger_gap())
r.gripper(0.04)
r.move_tcp(grasp + [0, 0, 0.10], R_V, 4.0)
r.move_tcp(grasp, R_V, 2.5)
gap = r.gripper(0.0)
r.move_tcp(grasp + [0, 0, 0.15], R_V, 2.5)
print('after lift fingers', r.finger_gap())
dest = np.array([-0.30 + R, -0.35, rimz - 0.024])
r.move_tcp(dest + [0, 0, 0.15], R_V, 4.0)
r.move_tcp(dest + [0, 0, 0.01], R_V, 2.5)
r.gripper(0.04)
r.move_tcp(dest + [0, 0, 0.12], R_V, 2.5)
print('DONE')
