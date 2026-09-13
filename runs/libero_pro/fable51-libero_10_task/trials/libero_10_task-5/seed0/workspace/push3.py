#!/usr/bin/env python3
"""Side push: closed fingers descend at (X,Y0,Z) then move along +y to Y1 in 1 cm steps (wrench-monitored), retreat up.
Usage: push3.py X Y0 Y1 [Z]"""
import sys, numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
X, Y0, Y1 = map(float, sys.argv[1:4]); Z = float(sys.argv[4]) if len(sys.argv) > 4 else 0.935
Q2 = Rot.from_matrix(np.stack([[-1,0,0],[0,1,0],[0,0,-1]], 1)).as_quat()   # hand x=-x, slide=y, z down
r = Robot("push3"); r.report("start")
r.gripper(0.0)
s = r.ik((X, Y0, 1.08), Q2, seed=r.arm_q()); assert s
r.move(s, 5.0, retries=3); r.report("above")
w0 = r.wrench(); seed = s
for z in np.arange(1.07, Z - 1e-6, -0.01):
    s = r.ik((X, Y0, z), Q2, seed=seed); assert s; seed = s
    r.move(s, 0.6, retries=1)
    dw = r.wrench() - w0
    if abs(dw[2]) > 3: print("descent contact at z", z, dw.round(2)); break
r.report("down"); w0 = r.wrench()
sgn = 1 if Y1 > Y0 else -1
for y in np.arange(Y0 + sgn*0.01, Y1 + sgn*1e-6, sgn*0.01):
    s = r.ik((X, y, Z), Q2, seed=seed); assert s; seed = s
    r.move(s, 0.8, retries=1)
    dw = r.wrench() - w0
    print(f"y={y:.3f} dF={dw[:3].round(2)}")
    if np.abs(dw[:2]).max() > 10: print("jam, stop"); break
p = r.report("pushed")
s = r.ik((p[0], p[1], 1.10), Q2, seed=seed); assert s; r.move(s, 2.0); r.report("up")
