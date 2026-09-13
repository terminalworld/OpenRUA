"""Carry hanging mug to clear area: up, then over to (-0.08,-0.22,1.16), same orientation."""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
r = Planner("step5")
tcp, R = r.solve_fk(r.joints()); tcp = tcp + TCP*R[:,2]
print("tcp now", np.round(tcp,3), "approach", np.round(R[:,2],3))
up = tcp.copy(); up[2] = 1.16
ok = r.move_line_tcp([up], R, avoid=True)
print("up ok", ok)
goal = np.array([-0.08, -0.22, 1.16])
q = best_ik(r, goal, R)
if q is None: sys.exit("no IK")
print("goto", np.round(q,3)); r.goto_joints(q)
tcp2 = r.tcp(); print("tcp after", np.round(tcp2,3), "fingers", np.round(r.fingers(),4))
