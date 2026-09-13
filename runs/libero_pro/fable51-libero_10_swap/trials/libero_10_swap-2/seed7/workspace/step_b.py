import numpy as np
from lib import Robot
r = Robot("stepb")
f0, t0 = r.wrench(); print("wrench before:", np.round(f0,3))
r.move_pose([-0.2095, 0.1965, 1.09], (1,0,0,0), sec=3.0)
f, t = r.wrench(); print("wrench @1.09:", np.round(f,3))
r.move_pose([-0.2095, 0.1965, 1.065], (1,0,0,0), sec=2.0)
f, t = r.wrench(); print("wrench @1.065:", np.round(f,3))
r.move_pose([-0.2095, 0.1965, 1.058], (1,0,0,0), sec=2.0)
f, t = r.wrench(); print("wrench @1.058:", np.round(f,3))
print("q:", np.round(r.arm_q(),4))
