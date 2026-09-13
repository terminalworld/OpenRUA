import numpy as np
from arm import *
a = Arm("place4")
print("open ->", a.gripper(0.04))
print("wrench", np.round(a.wrench(),2))
