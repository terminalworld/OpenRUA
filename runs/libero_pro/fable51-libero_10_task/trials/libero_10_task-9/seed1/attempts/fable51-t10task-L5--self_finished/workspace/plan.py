import numpy as np
from rob import *
A = np.radians(60)
# grasp orientation: fingers along world x, approach down & toward -y (palm leans +y)
R_G = hand_R([0, -np.sin(A), -np.cos(A)], [1, 0, 0])
# place orientation (after 180 deg yaw): approach down & toward +y (palm leans -y)
R_P = hand_R([0, np.sin(A), -np.cos(A)], [-1, 0, 0])
HX, HY, HZ = -0.124, -0.172, 0.935          # handle bar grasp point (TCP)
POSES = {
 'pregrasp': (np.array([HX, HY, HZ]) - 0.09 * R_G[:, 2], R_G),
 'grasp':    (np.array([HX, HY, HZ]), R_G),
 'lift':     (np.array([HX, HY, 1.13]), R_G),
 'rotated':  (np.array([HX, HY, 1.13]), R_P),
 'preinsert_hi': (np.array([-0.045, 0.13, 1.03]), R_P),
 'preinsert': (np.array([-0.045, 0.13, 0.995]), R_P),
 'insert':   (np.array([-0.045, 0.25, 0.995]), R_P),
 'lower':    (np.array([-0.045, 0.25, 0.982]), R_P),
}
