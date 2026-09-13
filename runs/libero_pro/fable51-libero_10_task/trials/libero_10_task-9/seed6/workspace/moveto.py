"""Cartesian TCP move keeping orientation: python3 moveto.py x y z [avoid=1]"""
import sys, numpy as np
from rob import *; from plan import Planner
r = Planner("moveto")
goal = np.array([float(v) for v in sys.argv[1:4]]); avoid = (sys.argv[4] != "0") if len(sys.argv) > 4 else True
tcp, R = r.tcp()
print("ok", r.move_line_tcp([goal], R, avoid=avoid), "tcp", np.round(r.tcp()[0],3))
