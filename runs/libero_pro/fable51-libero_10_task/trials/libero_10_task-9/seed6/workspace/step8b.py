"""Resume: approach along line to target and close."""
import sys, numpy as np
from rob import *; from plan import Planner
xw, yr, zc = map(float, sys.argv[1:4])
r = Planner("step8b")
appr = np.array([0, -np.cos(np.radians(45)), -np.sin(np.radians(45))])
R = R_from_axes(appr, [1, 0, 0])
target = np.array([xw, yr - 0.012, zc])
print("pre tcp", np.round(r.tcp()[0],3), "fingers", np.round(r.fingers(),4))
ok = r.move_line_tcp([target], R, avoid=False)
print("approach ok", ok, "tcp", np.round(r.tcp()[0],3))
r.gripper(0.0)
print("fingers", np.round(r.fingers(),4))
