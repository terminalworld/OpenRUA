import numpy as np
from rob import *
r = Rob()
r.gripper(0.04)
ok = r.move_tcp(0.0065, -0.2285, 0.60, sec=4)
print("ok", ok)
pos, q, tcp = r.fk_pose(); print("hand q", np.round(q,3))
r.shutdown()
