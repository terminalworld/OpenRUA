#!/usr/bin/env python3
"""Pick a flat box at (x,y) and lift it.  Usage: pick.py x y grasp_z [yaw_deg]"""
import sys
import numpy as np
from arm import *

x, y, gz = map(float, sys.argv[1:4])
yaw = math.radians(float(sys.argv[4])) if len(sys.argv) > 4 else 0.0
Q = q_down_yaw(yaw)
a = Arm()
print("start tcp", a.tcp()[0].round(4), "fingers", a.fingers(), flush=True)

print("open gripper", flush=True)
a.gripper(0.04)

print("pregrasp above", flush=True)
q_pre = a.ik((x, y, 0.60), Q)
a.move_q(q_pre, 4.0)
tp, _ = a.tcp(); print("  tcp", tp.round(4), flush=True)

print("descend", flush=True)
seed = q_pre
via = []
for i, z in enumerate([0.54, 0.49, 0.46]):
    seed = a.ik((x, y, z), Q, seed=seed)
    via.append((seed, 1.0 + i * 1.0))
q_g = a.ik((x, y, gz), Q, seed=seed)
a.move_q(q_g, 4.5, via=via)
tp, _ = a.tcp(); print("  tcp", tp.round(4), "target", (x, y, gz), flush=True)
err = np.array([x, y, gz]) - tp
if np.linalg.norm(err[:2]) > 0.004 or abs(err[2]) > 0.004:
    print("  correcting", err.round(4), flush=True)
    q_g = a.ik((x, y, gz), Q, seed=a.arm_q())
    a.move_q(q_g, 1.5)
    tp, _ = a.tcp(); print("  tcp", tp.round(4), flush=True)

print("close gripper", flush=True)
f = a.gripper(0.0)
print("  finger gap after close:", round(f[0] - f[1], 4), flush=True)

print("lift", flush=True)
q_up = a.ik((x, y, 0.62), Q, seed=a.arm_q())
a.move_q(q_up, 3.0)
tp, _ = a.tcp(); print("  tcp", tp.round(4), flush=True)
f = a.fingers(); print("  fingers after lift:", f, "gap", round(f[0] - f[1], 4), flush=True)
print("DONE", flush=True)
