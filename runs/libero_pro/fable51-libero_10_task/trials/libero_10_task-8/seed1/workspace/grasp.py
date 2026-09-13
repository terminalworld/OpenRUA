"""Grasp the lying moka pot across its bottom chamber (fingers perpendicular
to the pot axis), verify, lift.  Geometry from the wrist-camera scan."""
import math
import sys
import numpy as np
from robot import *

c = np.array([-0.1616, -0.0441])
d = np.array([0.741, 0.671])      # pot axis, +d = toward the base
n = np.array([-0.671, 0.741])     # finger opening direction
T_GRASP = float(sys.argv[1]) if len(sys.argv) > 1 else 0.045
Z_GRASP = float(sys.argv[2]) if len(sys.argv) > 2 else 0.916
NC = -0.0017

Q0 = (0.7071068, 0.7071068, 0, 0)
theta = math.atan2(-n[1], -n[0])           # hand x = -n
Qy = quat_mul((0, 0, math.sin(theta / 2), math.cos(theta / 2)), Q0)
print("theta deg", math.degrees(theta), "Qy", np.round(Qy, 4))
R = quat_R(*Qy)
print("hand x", R[:, 0].round(3), "hand z", R[:, 2].round(3))

g = c + T_GRASP * d + (NC + 0.00125) * n
print("grasp xy", g.round(4))

r = Robot()


def hold(k=1, sec=1.5):
    j = [r.joints()[q] for q in ARM]
    for _ in range(k):
        r.move_joints(j, sec)
    js = r.joints()
    return js["panda_finger_joint1"], js["panda_finger_joint2"]


# open gripper, confirm
r.gripper(GRIP["open_m"])
for i in range(40):
    f = hold()
    if min(f) > 0.039:
        break
print("fingers open", f)

goto_cl(r, [g[0], g[1], 1.03], Qy, 2.5, iters=3)
w0 = wrench(r)
print("baseline", w0[:3])
z = 1.03
for z in [1.0, 0.98, 0.965, 0.95, 0.94, 0.93, 0.922, Z_GRASP]:
    a = goto_cl(r, [g[0], g[1], z], Qy, 1.2, iters=2, tol=0.002)
    w = wrench(r)
    print("z", z, "actual", a.round(4), "dF", (w - w0)[:3])
    if np.abs((w - w0)[:3]).max() > 2.5:
        print("CONTACT during descent")
        break
print("at grasp height, fingers", hold())

r.gripper(GRIP["closed_m"])
prev = None
for i in range(60):
    f = hold()
    if prev is not None and abs(f[0] - prev[0]) < 0.0002 and abs(f[1] - prev[1]) < 0.0002:
        break
    prev = f
print("closed on", f, "gap", round(sum(f), 4))

w1 = wrench(r)
a = goto_cl(r, [g[0], g[1], Z_GRASP + 0.03], Qy, 1.5, iters=2)
f = hold()
w2 = wrench(r)
print("lifted 3cm: fingers", f, "dFz", (w2 - w1)[2], "dF vs base", (w2 - w0)[:3])
a = goto_cl(r, [g[0], g[1], 1.08], Qy, 2.0, iters=2)
f = hold()
print("lifted to 1.08: fingers", f, "wrench", wrench(r)[:3])
