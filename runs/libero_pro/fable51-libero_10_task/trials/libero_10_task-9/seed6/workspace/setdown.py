"""Lower TCP by dz (arg), open gripper, retreat up 0.10."""
import sys, numpy as np
from rob import *; from plan import Planner
dz = float(sys.argv[1]) if len(sys.argv) > 1 else 0.03
r = Planner("setdown")
tcp, R = r.solve_fk(r.joints()); tcp = tcp + TCP*R[:,2]
low = tcp.copy(); low[2] -= dz
print("lower ok", r.move_line_tcp([low], R, avoid=False))
r.gripper(0.04)
tcp, R = r.solve_fk(r.joints()); tcp = tcp + TCP*R[:,2]
up = tcp.copy(); up[2] += 0.10
print("up ok", r.move_line_tcp([up], R, avoid=False), "fingers", np.round(r.fingers(),4))
