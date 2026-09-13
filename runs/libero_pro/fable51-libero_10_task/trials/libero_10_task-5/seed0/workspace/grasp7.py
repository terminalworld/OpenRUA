#!/usr/bin/env python3
"""Top-down flat pinch on the SIDE wall (-y side) of the cup lying on its side, lift, pitch cup
upright about the pinch axis, perch it on the caddy corner.

Modes: plan | approach | close | lift | rotate | place X Y | retreat
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
from ikbest import ik_best, margin

def unit(v):
    v = np.array(v, float); return v / np.linalg.norm(v)

def quat(z, y):
    z = unit(z); y = unit(y); x = np.cross(y, z)
    return Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()

RIM_C = np.array([-0.240, 0.079, 0.942])
c = unit([-0.964, 0.217, 0.154])                 # cup axis bottom -> rim
n = unit(np.cross(c, [0, 0, 1.0]))               # horizontal, perpendicular to axis (+y side)
ch = unit([c[0], c[1], 0])                       # horizontal projection of axis
T_IN, R_W, DZ = 0.030, 0.047, 0.010               # depth inside rim, wall radius, pinch height above axis
p_ax = RIM_C - T_IN * c                          # axis point at pinch depth
r_h = np.sqrt(R_W**2 - DZ**2)
TCP = p_ax - r_h * n + [0, 0, DZ]                # -y side wall point
DESC = 0.049                                     # descend this far outside the rim (along ch), then insert
QA = quat([0, 0, -1], n)                         # hand down, slide along n
R_UP = Rot.from_rotvec(n * np.arccos(np.clip(c[2], -1, 1)))   # c -> z
QB = (R_UP * Rot.from_quat(QA)).as_quat()        # hand z -> +c (toward -x), slide still along n
CUP_OFF = -r_h * n[:2]                           # TCP_xy - cup centre_xy when upright
LIFT_Z = 1.16

r = Robot("grasp7")

def go(q, secs, tag, retries=3, tol=0.02):
    r.move(q, secs, retries=retries)
    err = np.abs(np.array(r.arm_q()) - np.array(q)).max()
    p = r.report(tag)
    print(f"   joint err after move {err:.4f}")
    if err > tol:
        print("   !!! move did not reach goal"); sys.exit(1)
    return p

mode = sys.argv[1]
r.report("start")
print("n", n.round(3), "TCP", TCP.round(4), "QB hand z", Rot.from_quat(QB).apply([0, 0, 1]).round(3), "CUP_OFF", CUP_OFF.round(4))
if mode == "plan":
    for tag, pos, qt in [("grasp", TCP, QA), ("desc", TCP - DESC * ch, QA), ("hi", TCP - DESC * ch + [0, 0, 0.20], QA),
                         ("lifted", TCP + [0, 0, LIFT_Z - TCP[2]], QA), ("liftedB", TCP + [0, 0, LIFT_Z - TCP[2]], QB),
                         ("perchB", np.array([-0.458, -0.082, 1.16]) + [CUP_OFF[0], CUP_OFF[1], 0], QB)]:
        b, s = ik_best(r, pos, qt, n=12)
        print(tag, "margin", None if b is None else round(margin(b), 2), None if b is None else np.round(b, 2))
elif mode == "approach":
    r.gripper(0.04)
    qg, _ = ik_best(r, TCP, QA, n=30); assert qg is not None
    print("grasp cfg", np.round(qg, 2), "margin", round(margin(qg), 2))
    qd = r.ik(TCP - DESC * ch, QA, seed=qg); assert qd
    qhi = r.ik(TCP - DESC * ch + [0, 0, 0.20], QA, seed=qd); assert qhi
    print("desc", np.round(qd, 2), "hi", np.round(qhi, 2))
    go(qhi, 6.0, "hi", retries=4)
    base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    seed = qhi
    for dz in (0.14, 0.09, 0.05, 0.03, 0.015, 0.0):        # descend outside the rim
        s = r.ik(TCP - DESC * ch + [0, 0, dz], QA, seed=seed); assert s
        seed = s; r.move(s, 1.2, retries=1)
        dw = r.wrench() - base_w; r.report(f"desc dz={dz:.3f}"); print("   dwrench", dw.round(2))
        if np.abs(dw[:3]).max() > 3.0:
            print("   contact during descent! abort"); sys.exit(1)
    for d in np.arange(DESC - 0.01, -1e-6, -0.01):        # insert along axis
        s = r.ik(TCP - d * ch, QA, seed=seed); assert s
        seed = s; r.move(s, 0.8, retries=1)
        dw = r.wrench() - base_w; r.report(f"insert d={d:.3f}"); print("   dwrench", dw.round(2))
        if np.abs(dw[:3]).max() > 3.0:
            print("   contact during insertion! stopping"); break
    f = r.gripper(0.0)
    print("after close: fingers", np.round(f, 4), "gap", round(f[0] - f[1], 4))
elif mode == "lift":
    w0 = r.wrench(); p, qt, _ = r.tcp(); seed = r.arm_q()
    for dz in (0.02, 0.05, 0.10, LIFT_Z - p[2]):
        s = r.ik(p + [0, 0, dz], qt, seed=seed); assert s
        seed = s; r.move(s, 1.5, retries=2)
        r.report(f"lift {dz:.2f}")
        print("   fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))
elif mode == "rotate":
    p, qt, _ = r.tcp(); seed = r.arm_q()
    q0 = Rot.from_quat(qt); q1 = Rot.from_quat(QB)
    rv = (q1 * q0.inv()).as_rotvec(); N = 5
    for i in range(1, N + 1):
        qi = (Rot.from_rotvec(rv * i / N) * q0).as_quat()
        s = r.ik(p, qi, seed=seed); assert s, f"IK rot {i}"
        if np.abs(np.array(s) - np.array(seed)).max() > 0.6:
            print("   branch jump; abort"); sys.exit(1)
        seed = s; r.move(s, 2.0, retries=2)
        r.report(f"rot {i}/{N}")
        print("   fingers", np.round(r.fingers(), 4), "wrench", r.wrench().round(2))
elif mode == "place":
    X, Y = float(sys.argv[2]), float(sys.argv[3])
    txy = np.array([X, Y]) + CUP_OFF
    seed = r.arm_q(); base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    s = r.ik((txy[0], txy[1], 1.21), QB, seed=seed); assert s
    go(s, 6.0, "over perch"); seed = s
    z = 1.20
    while z >= 1.10:
        s = r.ik((txy[0], txy[1], z), QB, seed=seed); assert s
        seed = s; r.move(s, 1.0, retries=1)
        w = r.wrench(); dw = w - base_w
        r.report(f"z={z:.3f}")
        print("   dwrench", dw.round(2), "fingers", np.round(r.fingers(), 4))
        if abs(dw[2]) > 1.5 or np.abs(dw[3:5]).max() > 0.4:
            print("   contact"); break
        z -= 0.01
    r.gripper(0.04)
    print("released; fingers", np.round(r.fingers(), 4))
elif mode == "retreat":
    p, qt, _ = r.tcp(); seed = r.arm_q()
    for dz in (0.04, 0.08):
        s = r.ik(p + [0, 0, dz], qt, seed=seed); assert s
        seed = s; r.move(s, 1.5, retries=2); r.report(f"up {dz}")
    s = r.ik(p + [0.12, 0.15, 0.20], qt, seed=seed)
    if s: r.move(s, 3.0, retries=2); r.report("away")
