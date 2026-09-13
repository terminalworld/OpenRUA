import numpy as np
from lib import Robot
r = Robot("steph")
quat = (0.8315, 0.5556, 0, 0)
f0 = r.wrench()[0]; print("wrench0", np.round(f0,3))
for z in [1.13, 1.103]:
    r.move_pose([-0.0294, -0.237, z], quat, sec=1.5)
    f = r.wrench()[0]; print(f"wrench @hand z={z}:", np.round(f,3), "delta", np.round(f-f0,3))
    if np.abs(f - f0).max() > 2.0:
        print("CONTACT detected, stopping descent"); break
