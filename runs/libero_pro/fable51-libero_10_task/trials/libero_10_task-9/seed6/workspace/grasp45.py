"""Grasp vertical handle bar from -y with 45deg lean: grasp45.py bx by bz [lift]"""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
bx, by, bz = map(float, sys.argv[1:4]); lift = float(sys.argv[4]) if len(sys.argv) > 4 else 0.03
r = Planner("grasp45")
a = np.array([0, np.sqrt(.5), -np.sqrt(.5)]); R = R_from_axes(a, [1,0,0])
target = np.array([bx, by, bz]); pre = target - 0.08*a
r.gripper(0.04)
q = best_ik(r, pre, R); assert q is not None, "no IK for pregrasp"
print("goto pregrasp"); r.goto_joints(q); print("tcp", r.tcp()[0].round(4))
ok = r.move_line_tcp([target], R, avoid=False); print("approach ok", ok)
r.gripper(0.0)
ok = r.move_line_tcp([target + [0,0,lift]], R, avoid=False); print("lift ok", ok, "fingers", np.round(r.fingers(),4))
