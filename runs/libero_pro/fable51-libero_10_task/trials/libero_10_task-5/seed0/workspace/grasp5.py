#!/usr/bin/env python3
"""Horizontal rim-wall pinch of the cup lying on its side, then rotate upright and perch.
Modes: approach | lift | rotate | place X Y
"""
import sys, numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
from ikbest import ik_best, margin

def unit(v): v = np.array(v, float); return v / np.linalg.norm(v)
def quat(z, y):
    z = unit(z); y = unit(y); x = np.cross(y, z)
    return Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()

RIM_C = np.array([-0.240, 0.079, 0.942])          # rim centre (world)
A = unit([0.964, -0.217, -0.154])                 # approach = into the opening (rim -> bottom)
U = unit(np.array([0, 0, 1.0]) - np.dot([0, 0, 1.0], A) * A)   # "up" perpendicular to A
S = unit(np.cross(U, A))                          # side toward +y (handle side)
ANG = np.radians(60.0)
SLIDE = unit(np.sin(ANG) * S + np.cos(ANG) * U)   # outer finger direction
R_WALL, INSIDE = 0.0475, 0.02
TCP = RIM_C + INSIDE * A + R_WALL * SLIDE
QA = quat(A, SLIDE)
# after rotation: hand down, slide horizontal
SLIDE2 = unit([0.678, 0.734, 0.0])
QB = quat([0, 0, -1], SLIDE2)
CUP_OFF2 = R_WALL * SLIDE2[:2]                    # cup centre = TCP_xy - CUP_OFF2

r = Robot("grasp5")
mode = sys.argv[1]
r.report("start")
if mode == "approach":
    print("A", A.round(3), "SLIDE", SLIDE.round(3), "TCP", TCP.round(4))
    r.gripper(0.04)
    p, q, _ = r.tcp()
    s = r.ik(p + [0, 0, 1.20 - p[2]], q, seed=r.arm_q()); assert s
    r.move(s, 3.0, retries=2); r.report("up")
    qg, sols = ik_best(r, TCP, QA, n=30); assert qg is not None
    print("grasp config", qg.round(2), "margin", round(margin(qg), 2))
    qpre = r.ik(TCP - 0.12 * A, QA, seed=list(qg)); assert qpre, "IK pre"
    print("pre config", np.round(qpre, 2), "dist to grasp", round(float(np.abs(np.array(qpre) - qg).max()), 2))
    qhi = r.ik(TCP - 0.12 * A + [0, 0, 0.15], QA, seed=qpre); assert qhi, "IK hi"
    print("hi config", np.round(qhi, 2))
    r.move(qhi, 7.0, retries=4); r.report("hi")
    r.move(qpre, 4.0, retries=3); r.report("pre")
    w0 = r.wrench(); seed = qpre
    for d in np.arange(0.11, -1e-6, -0.01):
        s = r.ik(TCP - d * A, QA, seed=seed); assert s, f"IK d={d}"; seed = s
        r.move(s, 0.7, retries=1)
        dw = r.wrench() - w0
        r.report(f"d={d:.2f}"); print("   dF", dw[:3].round(2))
        if np.abs(dw[:3]).max() > 3.0:
            print("   contact! stop"); break
    f = r.gripper(0.0)
    print("closed: fingers", np.round(f, 4), "gap", round(f[0] - f[1], 4))
elif mode == "lift":
    p, q, _ = r.tcp(); w0 = r.wrench(); seed = r.arm_q()
    for dz in (0.03, 0.08, 0.15, 0.21):
        s = r.ik(p + [0, 0, dz], q, seed=seed); assert s; seed = s
        r.move(s, 2.0, retries=2)
        print(f"dz={dz}: fingers {np.round(r.fingers(),4)} dF {(r.wrench()-w0)[:3].round(2)}")
    r.report("lifted")
elif mode == "rotate":
    p, q, _ = r.tcp(); seed = r.arm_q()
    key = Rot.from_quat([QA, QB])
    for t in (0.33, 0.66, 1.0):
        qi = Rot.from_quat(QA) * (Rot.from_quat(QA).inv() * Rot.from_quat(QB)) ** t
        s = r.ik(p, qi.as_quat(), seed=seed); assert s, f"IK rot t={t}"; seed = s
        r.move(s, 3.0, retries=3)
        r.report(f"rot t={t}"); print("   fingers", np.round(r.fingers(), 4))
elif mode == "place":
    X, Y = float(sys.argv[2]), float(sys.argv[3])
    txy = np.array([X, Y]) + CUP_OFF2
    p, q, _ = r.tcp(); seed = r.arm_q()
    s = r.ik((txy[0], txy[1], 1.20), QB, seed=seed); assert s; seed = s
    r.move(s, 6.0, retries=3); r.report("over perch")
    w0 = r.wrench(); print("baseline", w0[:3].round(2))
    z = 1.19
    while z >= 1.10:
        s = r.ik((txy[0], txy[1], z), QB, seed=seed); assert s; seed = s
        r.move(s, 0.8, retries=1)
        dw = r.wrench() - w0
        r.report(f"z={z:.3f}"); print("   dF", dw[:3].round(2), "fingers", np.round(r.fingers(), 4))
        if abs(dw[2]) > 1.5 or np.abs(dw[3:5]).max() > 0.4:
            print("   contact -> release"); break
        z -= 0.01
    r.gripper(0.04)
    s = r.ik((txy[0], txy[1], 1.25), QB, seed=seed); assert s; r.move(s, 2.0, retries=2)
    r.report("retreated")
