#!/usr/bin/env python3
"""Pinch the lying cup's bottom end from above. TCP (X,Y), slide axis n=(NX,NY), fingertip Z.
Descend with wrench monitoring; on contact shift along lateral force and retry. Then close, lift LIFT, report.
Usage: pinchbottom.py X Y NX NY Z LIFT"""
import sys, numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
X, Y, NX, NY, ZT, LIFT = map(float, sys.argv[1:7])
n = np.array([NX, NY, 0.0]); n /= np.linalg.norm(n)
z = np.array([0, 0, -1.0]); x = np.cross(n, z)
Q = Rot.from_matrix(np.stack([x, n, z], 1)).as_quat()
r = Robot("pinchbottom"); r.report("start")
r.gripper(0.04)
tcp = np.array([X, Y, 1.06])
s = r.ik(tcp, Q, seed=r.arm_q()); assert s
r.move(s, 5.0, retries=3); r.report("above")
w0 = r.wrench(); seed = s
for attempt in range(4):
    ok = True
    for zz in np.arange(1.05, ZT - 1e-6, -0.01):
        s = r.ik((tcp[0], tcp[1], zz), Q, seed=seed); assert s; seed = s
        r.move(s, 0.5, retries=1)
        dw = r.wrench() - w0
        if abs(dw[2]) > 2.5:
            lat = np.dot(dw[:3], n)
            print(f"descent contact z={zz:.3f} dF={dw[:3].round(2)} lateral={lat:.2f}")
            ok = False; break
    if ok: break
    # retreat and shift
    s = r.ik((tcp[0], tcp[1], 1.02), Q, seed=seed); r.move(s, 1.0); seed = s
    shift = 0.006 * np.sign(lat) if abs(lat) > 0.3 else 0.006
    tcp[:2] += shift * n[:2]; print("shift to", tcp.round(4))
p = r.report("down")
f = r.gripper(0.0); print("fingers", np.round(f, 4), "gap", round(f[0]-f[1], 4))
if f[0] - f[1] < 0.05:
    print("missed -> open"); r.gripper(0.04); sys.exit(1)
w1 = r.wrench()
s = r.ik(p + [0, 0, LIFT], Q, seed=seed); assert s; r.move(s, 2.0, retries=2)
print("after lift fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w1).round(2))
r.report("lifted")
