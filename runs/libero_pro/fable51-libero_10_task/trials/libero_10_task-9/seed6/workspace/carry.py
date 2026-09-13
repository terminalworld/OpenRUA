"""Cartesian TCP moves keeping orientation: carry.py dx dy dz [dx dy dz ...] (relative steps)."""
import sys, numpy as np
from rob import *; from plan import Planner
r = Planner("carry"); v = [float(a) for a in sys.argv[1:]]
for i in range(0, len(v), 3):
    tcp, R = r.tcp(); tgt = tcp + np.array(v[i:i+3])
    ok = r.move_line_tcp([tgt], R, avoid=False); print("step", v[i:i+3], "ok", ok, "fingers", np.round(r.fingers(),4))
