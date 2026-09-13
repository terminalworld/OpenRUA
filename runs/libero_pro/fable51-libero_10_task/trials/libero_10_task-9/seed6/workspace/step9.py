"""Rim grasp of lying mug at its upper-left (45deg) rim point.
Args: x_c y_rim z_c. finger axis f=(-.707,0,.707), approach a=(-.5,-.707,-.5)."""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
xc, yr, zc = map(float, sys.argv[1:4])
r = Planner("step9")
f = np.array([-1, 0, 1]) / np.sqrt(2)
a = np.array([-0.5, -np.sqrt(0.5), -0.5])
R = R_from_axes(a, f)
target = np.array([xc - 0.0435*0.707, yr - 0.02, zc + 0.0435*0.707])
pre = target - 0.08 * a
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
