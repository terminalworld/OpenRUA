"""Rim grasp of lying mug: 45deg-pitched fingers from +y, pinch rim wall at its -x point.
Args: x_wall y_rim z_c  (wall centre x, rim plane y, mug axis height)."""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
xw, yr, zc = map(float, sys.argv[1:4])
r = Planner("step8")
appr = np.array([0, -np.cos(np.radians(45)), -np.sin(np.radians(45))])
R = R_from_axes(appr, [1, 0, 0])
target = np.array([xw, yr - 0.012, zc])
pre = target - 0.08 * appr
print("target", np.round(target,3), "pre", np.round(pre,3))
if abs(r.fingers()[0]) < 0.035: r.gripper(0.04)
q = best_ik(r, pre, R)
if q is None: sys.exit("no IK for pre")
r.goto_joints(q)
print("pre tcp", np.round(r.tcp()[0],3))
ok = r.move_line_tcp([target], R, avoid=False)
print("approach ok", ok, "tcp", np.round(r.tcp()[0],3))
r.gripper(0.0)
print("fingers", np.round(r.fingers(),4))
