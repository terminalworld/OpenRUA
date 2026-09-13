import sys, numpy as np
from rob import *; from plan import Planner
dz = float(sys.argv[1])
r = Planner("lift")
tcp, R = r.tcp(); up = tcp.copy(); up[2] += dz
print("lift ok", r.move_line_tcp([up], R, avoid=False), "tcp", np.round(r.tcp()[0],3), "fingers", np.round(r.fingers(),4))
