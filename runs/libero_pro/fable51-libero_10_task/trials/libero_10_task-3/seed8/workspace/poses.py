import numpy as np
from robot import rot_from_axes
c20, s20 = np.cos(np.radians(20)), np.sin(np.radians(20))
R_GRASP = rot_from_axes([c20, 0, -s20], [0, -1, 0])           # approach +x tilted 20 deg down, close along y
c45 = np.sqrt(0.5)
R_PLACE = rot_from_axes([0, c45, -c45], [0, c45, c45])         # approach +y 45deg down, close along tilted z
c40, s40 = np.cos(np.radians(40)), np.sin(np.radians(40))
R_PUSH = rot_from_axes([0, s40, -c40], [1, 0, 0])              # approach +y 40deg from vertical, close along x
BOTTLE = np.array([-0.148, 0.053])
PRE_GRASP = np.array([-0.26, BOTTLE[1], 0.955])
GRASP = np.array([BOTTLE[0], BOTTLE[1], 0.955])
LIFT = np.array([BOTTLE[0], BOTTLE[1], 1.25])
REORIENT = np.array([-0.15, 0.10, 1.25])
PRE_PLACE = np.array([0.005, 0.15, 1.15])
PLACE = np.array([0.005, 0.15, 1.02])
RETREAT = np.array([0.005, 0.10, 1.25])
PRE_PUSH = np.array([0.06, 0.05, 1.10])
PUSH0 = np.array([0.06, 0.05, 0.965])
PUSH1 = np.array([0.06, 0.207, 0.965])
