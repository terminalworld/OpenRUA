from robot import *
TOP = np.array([1.0, 0, 0, 0])
MUG = np.array([-0.188, 0.033]); RIM_Z = 0.575; R_MUG = 0.044
GRASP_XY = MUG + [0, R_MUG]
PLATE = np.array([0.145, 0.013])
PUD = np.array([-0.020, 0.100]); PUD_Q = quat_mul(quat_z(math.radians(11)), TOP)
