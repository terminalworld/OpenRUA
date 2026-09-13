"""Grasp vertical handle bar from -y with given lean (deg): graspbar.py bx by bz lean [lift]"""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
bx, by, bz, lean = map(float, sys.argv[1:5]); lift = float(sys.argv[5]) if len(sys.argv) > 5 else 0.03
r = Planner("graspbar")
b = np.radians(lean); a = np.array([0, np.sin(b), -np.cos(b)]); R = R_from_axes(a, [1,0,0])
target = np.array([bx, by, bz]); pre = target - 0.08*a
r.gripper(0.04)
q = best_ik(r, pre, R, tries=30); assert q is not None, "no IK for pregrasp"
print("goto pregrasp"); r.goto_joints(q); print("tcp", r.tcp()[0].round(4))
ok = r.move_line_tcp([target], R, avoid=False); print("approach ok", ok)
r.gripper(0.0)
ok = r.move_line_tcp([target + [0,0,lift]], R, avoid=False); print("lift ok", ok, "fingers", np.round(r.fingers(),4))
