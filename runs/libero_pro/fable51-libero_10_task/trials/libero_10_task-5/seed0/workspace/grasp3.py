#!/usr/bin/env python3
"""Grasp the fallen cup by clamping its handle loop along the loop's long axis.

Loop centre c, loop axis p (tilted 45 deg in the x-z plane), hand approach z perpendicular to p
coming from +x/above. Fingers open to +-4 cm about c along p, loop is ~6.1 cm long.
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot

c = np.array([-0.360, -0.225, 1.008])
p = np.array([-0.716, 0, 0.698]); p /= np.linalg.norm(p)
z = np.array([-0.698, 0, -0.716]); z /= np.linalg.norm(z)
x = np.cross(p, z)
Q = Rot.from_matrix(np.stack([x, p, z], 1)).as_quat()
BACK, STEP = 0.08, 0.01
SEED = [-0.76, 0.59, 0.45, -1.77, -0.44, 1.51, 2.23]

r = Robot("grasp3")
r.report("start")
r.gripper(0.04)
pre = r.ik(c - BACK * z, Q, seed=SEED); assert pre, "IK pre"
r.move(pre, 5.0)
r.report("pre")
base_w = r.wrench(); print("baseline wrench", base_w.round(2))

seed = pre
for d in np.arange(BACK - STEP, -1e-6, -STEP):
    s = r.ik(c - d * z, Q, seed=seed); assert s, f"IK d={d}"
    seed = s
    r.move(s, 0.8, retries=1)
    w = r.wrench(); dw = w - base_w
    r.report(f"d={d:.3f}")
    print("   dwrench", dw.round(2))
    if np.abs(dw[:3]).max() > 2.5:
        print("   contact! stopping approach"); break

f = r.gripper(0.0)
w = r.wrench(); print("after close: fingers", np.round(f, 4), "dwrench", (w - base_w).round(2))
if f[0] > 0.015:
    print("loop clamped (fingers stopped early) -> lifting")
    s = r.ik(r.tcp()[0] + [0, 0, 0.15], Q, seed=r.arm_q()); assert s
    r.move(s, 3.0)
    w = r.wrench(); print("after lift: fingers", np.round(r.fingers(), 4), "dwrench", (w - base_w).round(2))
    r.report("lifted")
else:
    print("fingers closed fully -> missed")
