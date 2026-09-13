#!/usr/bin/env python3
"""Pinch the cup's bottom end from above (slide along y), lift slightly, yaw the hand by ANG deg
about the vertical, lower, release. Usage: pivot.py X Y ZTIP ANG"""
import sys, numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
X, Y, ZT, ANG = map(float, sys.argv[1:5])
def quat_yaw(deg):
    R0 = np.stack([[-1,0,0],[0,1,0],[0,0,-1]], 1)   # hand x=-x, slide=y, z down
    return Rot.from_matrix(Rot.from_euler('z', deg, degrees=True).as_matrix() @ R0).as_quat()
r = Robot("pivot"); r.report("start")
r.gripper(0.04)
s = r.ik((X, Y, 1.08), quat_yaw(0), seed=r.arm_q()); assert s
r.move(s, 5.0, retries=3); r.report("above")
w0 = r.wrench(); seed = s
for z in np.arange(1.07, ZT - 1e-6, -0.01):
    s = r.ik((X, Y, z), quat_yaw(0), seed=seed); assert s; seed = s
    r.move(s, 0.6, retries=1)
    dw = r.wrench() - w0
    if abs(dw[2]) > 3: print("descent contact at z", z, dw.round(2)); break
r.report("down")
f = r.gripper(0.0); print("fingers", np.round(f, 4), "gap", round(f[0]-f[1], 4))
if f[0] - f[1] < 0.05:
    print("missed -> open and stop"); r.gripper(0.04); sys.exit(1)
p = r.tcp()[0]
s = r.ik((X, Y, p[2] + 0.01), quat_yaw(0), seed=seed); assert s; r.move(s, 1.0); seed = s
step = 15.0 if ANG > 0 else -15.0
angs = list(np.arange(step, ANG + step/2, step))
if abs(angs[-1]) > abs(ANG): angs[-1] = ANG
for a in angs:
    s = r.ik((X, Y, p[2] + 0.01), quat_yaw(a), seed=seed); assert s, f"IK yaw {a}"; seed = s
    r.move(s, 1.5, retries=2)
    print(f"yaw {a:.0f}: fingers {np.round(r.fingers(),4)}")
s = r.ik((X, Y, p[2]), quat_yaw(ANG), seed=seed); assert s; r.move(s, 1.0)
r.gripper(0.04)
s = r.ik((X, Y, 1.10), quat_yaw(ANG), seed=seed); assert s; r.move(s, 2.0); r.report("up")
