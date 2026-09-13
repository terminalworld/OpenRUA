import numpy as np
from rob import *
r = Rob()
ok = r.move_tcp(0.0065, -0.2285, 0.60, sec=3)
pos, q, tcp = r.fk_pose(); print("ok", ok, "hand q", np.round(q,3), "joints", np.round(r.arm_q(),3))
r.shutdown()
