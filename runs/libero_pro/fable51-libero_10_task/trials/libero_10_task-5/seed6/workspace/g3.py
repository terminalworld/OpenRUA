import numpy as np
from ctl import *
r=Robot("g3")
f=r.gripper(0.0)
r.spin(10); print("fingers after",r.finger())
