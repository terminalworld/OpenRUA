import sys, numpy as np
from rob import *; from plan import Planner
r = Planner("insert"); R = R_from_axes([0,1,0],[1,0,0])
wps = [[-0.06, 0.12, 1.06], [-0.06, 0.12, 1.02]]
for w in wps:
    ok = r.move_line_tcp([w], R, avoid=True); print("to", w, "ok", ok, "fingers", np.round(r.fingers(),4))
    if not ok: sys.exit("move failed")
