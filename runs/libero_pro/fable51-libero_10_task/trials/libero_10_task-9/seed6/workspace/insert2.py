import sys, numpy as np
from rob import *; from plan import Planner; from attach import attach_mug
r = Planner("insert2"); R = R_from_axes([0,1,0],[1,0,0])
tcp,_ = r.tcp(); attach_mug(r, tcp + np.array([0, 0.0725, -0.010]), radius=0.048, height=0.114, axis_world=(0,0,1))
for w, avoid in [([-0.06, 0.12, 1.032], True), ([-0.06, 0.29, 1.032], True)]:
    ok = r.move_line_tcp([w], R, avoid=avoid); print("to", w, "ok", ok, "fingers", np.round(r.fingers(),4))
    if not ok: sys.exit("move failed")
