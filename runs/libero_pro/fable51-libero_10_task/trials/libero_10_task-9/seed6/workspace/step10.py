"""Grasp handle bar of lying mug from above. Args: bx by bz tx ty (bar centre, mug axis dir toward rim)."""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
bx, by, bz, tx, ty = map(float, sys.argv[1:6])
r = Planner("step10")
t = np.array([tx, ty, 0]); t /= np.linalg.norm(t)
perp = np.array([-t[1], t[0], 0])
lean = np.radians(20)
a = np.sin(lean)*t + np.array([0, 0, -np.cos(lean)])      # fingers point down and toward rim; hand leans toward base
R = R_from_axes(a, perp)
target = np.array([bx, by, bz]); pre = target - 0.07*a
print("target", target.round(3), "pre", pre.round(3), "a", a.round(3))
if abs(r.fingers()[0]) < 0.035: r.gripper(0.04)
q = best_ik(r, pre, R)
if q is None: sys.exit("no IK")
r.goto_joints(q); print("pre tcp", np.round(r.tcp()[0],3))
print("approach ok", r.move_line_tcp([target], R, avoid=False))
r.gripper(0.0); print("fingers", np.round(r.fingers(),4))
up = target.copy(); up[2] += 0.05
print("lift ok", r.move_line_tcp([up], R, avoid=False), "fingers", np.round(r.fingers(),4))
