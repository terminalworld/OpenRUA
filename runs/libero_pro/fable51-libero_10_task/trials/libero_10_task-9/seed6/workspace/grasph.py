"""Horizontal bar grasp from -y (fingers point +y, pads along x): grasph.py bx by bz [lift]"""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
bx, by, bz = map(float, sys.argv[1:4]); lift = float(sys.argv[4]) if len(sys.argv) > 4 else 0.03
r = Planner("grasph")
R = R_from_axes([0,1,0], [1,0,0]); a = R[:,2]
target = np.array([bx, by, bz]); pre = target - 0.08*a
r.gripper(0.04)
q = best_ik(r, pre, R, tries=30); assert q is not None, "no IK for pregrasp"
print("goto pregrasp"); ok = r.goto_joints(q); print("pregrasp ok", ok, "tcp", r.tcp()[0].round(4))
ok = r.move_line_tcp([target], R, avoid=False); print("approach ok", ok)
r.gripper(0.0)
ok = r.move_line_tcp([target + [0,0,lift]], R, avoid=False); print("lift ok", ok, "fingers", np.round(r.fingers(),4))
