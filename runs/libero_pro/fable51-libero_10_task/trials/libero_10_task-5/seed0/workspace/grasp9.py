#!/usr/bin/env python3
"""Rim-wall pinch (approach along cup axis, optional pitch; pinch point ROLL deg from top toward +y) of the cup lying on its side, then upright it and perch it on the caddy.

Modes: plan | approach | lift | rotate | place X Y | retreat
Cup: rim centre RIM_C, axis c (bottom->rim).  Hand approaches into the opening along A (= -c pitched
PITCH deg downward), fingers pinch the top wall (slide axis in the vertical plane through c).
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
from ikbest import ik_best, margin

mode = sys.argv[1]
mode_has_pitch = mode in ("plan", "approach")

def unit(v):
    v = np.array(v, float); return v / np.linalg.norm(v)

def quat(z, y):
    z = unit(z); y = unit(y); x = np.cross(y, z)
    return Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()

RIM_C = np.array([-0.219, 0.088, 0.943])  # re-measured after bump
c = unit([-0.964, 0.150, 0.150])                    # cup axis bottom -> rim
U = unit(np.array([0, 0, 1.0]) - c[2] * c)          # "up" perpendicular to c
PITCH = np.radians(float(sys.argv[2]) if len(sys.argv) > 2 and mode_has_pitch else 0.0)
SGN = -1
PSI = -135.0                                        # extra yaw of the uprighting rotation
ROT_P = np.array([-0.21, 0.15, 1.10])               # TCP position where the cup is uprighted
INSIDE, WALL_R = 0.020, 0.0475
ROLL = np.radians(-50.0)                            # pinch point rotated from top toward -y side (away from handle & caddy)
S = unit(np.cross(U, -c))                           # side direction (toward +y)
W = unit(np.sin(ROLL) * S + np.cos(ROLL) * U)       # radial direction of pinch point
A = unit(-c * np.cos(PITCH) - W * np.sin(PITCH))     # approach direction
SL = unit(-c * np.sin(PITCH) + W * np.cos(PITCH))    # inner -> outer finger
TCP = RIM_C - INSIDE * c + WALL_R * W
QA = quat(A, SGN * SL)
# rotation that brings the cup upright (c -> +z) about a horizontal axis
ax = unit(np.cross(c, [0, 0, 1.0])); ang = np.arccos(np.clip(c[2], -1, 1))
R_UP = Rot.from_euler("z", PSI, degrees=True) * Rot.from_rotvec(ax * ang)
QB = (R_UP * Rot.from_quat(QA)).as_quat()
H = R_UP.apply(W)                                   # world direction TCP lies from the cup axis when upright
CUP_OFF2 = WALL_R * H[:2]                           # cup centre = TCP_xy - CUP_OFF2
BACK, STEP, LIFT_Z = 0.10, 0.01, 1.15

r = Robot("grasp9")

def go(q, secs, tag, retries=3, tol=0.02):
    r.move(q, secs, retries=retries)
    err = np.abs(np.array(r.arm_q()) - np.array(q)).max()
    p = r.report(tag)
    print(f"   joint err after move {err:.4f}")
    if err > tol:
        print("   !!! move did not reach goal"); sys.exit(1)
    return p

def check(q, pos, quat_, tag):
    p, qq, _ = r.fk_tcp(q) if hasattr(r, "fk_tcp") else (None, None, None)

r.report("start")
print("A", A.round(3), "SL", SL.round(3), "TCP", TCP.round(4), "QB hand z", Rot.from_quat(QB).apply([0, 0, 1]).round(3),
      "H", H.round(3))
if mode == "plan":
    for tag, pos, qt in [("grasp", TCP, QA), ("pre", TCP - BACK * A, QA), ("hi", TCP - BACK * A + [0, 0, 0.15], QA),
                         ("lifted", TCP + [0, 0, LIFT_Z - TCP[2]], QA), ("liftedB", TCP + [0, 0, LIFT_Z - TCP[2]], QB),
                         ("perchB", np.array([-0.458, -0.082, 1.16]) + [CUP_OFF2[0], CUP_OFF2[1], 0], QB)]:
        b, s = ik_best(r, pos, qt, n=12)
        print(tag, "margin", None if b is None else round(margin(b), 2), None if b is None else np.round(b, 2))
elif mode == "approach":
    r.gripper(0.04)
    qg, _ = ik_best(r, TCP, QA, n=30); assert qg is not None
    print("grasp cfg", np.round(qg, 2), "margin", round(margin(qg), 2))
    qpre = r.ik(TCP - BACK * A, QA, seed=qg); assert qpre
    qhi = r.ik(TCP - BACK * A + [0, 0, 0.15], QA, seed=qpre); assert qhi
    for nm, q in (("pre", qpre), ("hi", qhi)):
        print(nm, np.round(q, 2), "dq from grasp", np.abs(np.array(q) - np.array(qg)).max().round(2))
    q0 = r.arm_q()
    qlift = r.ik(r.tcp()[0] + [0, 0, 1.20 - r.tcp()[0][2]], r.tcp()[1], seed=q0)
    if qlift: go(qlift, 4.0, "lift-current")
    go(qhi, 7.0, "hi", retries=4)
    go(qpre, 4.0, "pre")
    base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    seed = qpre
    for d in np.arange(BACK - STEP, -1e-6, -STEP):
        s = r.ik(TCP - d * A, QA, seed=seed); assert s, f"IK d={d}"
        if np.abs(np.array(s) - np.array(seed)).max() > 0.4:
            print("   branch jump, abort"); sys.exit(1)
        seed = s
        r.move(s, 0.8, retries=1)
        dw = r.wrench() - base_w
        r.report(f"d={d:.3f}")
        print("   dwrench", dw.round(2))
        if np.abs(dw[:3]).max() > 3.0:
            print("   contact! stopping approach"); break
    f = r.gripper(0.0)
    print("after close: fingers", np.round(f, 4), "gap", round(f[0] - f[1], 4))
elif mode == "lift":
    w0 = r.wrench(); p, qt, _ = r.tcp(); seed = r.arm_q()
    for dz in (0.03, 0.06):
        s = r.ik(p + [0, 0, dz], qt, seed=seed); assert s
        seed = s; r.move(s, 1.5, retries=2)
        r.report(f"lift {dz:.2f}")
        print("   fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))
    p1 = p + [0, 0, 0.06]
    for k in range(1, 4):                          # translate to ROT_P in 3 steps
        pk = p1 + (ROT_P - p1) * k / 3
        s = r.ik(pk, qt, seed=seed); assert s, "IK translate"
        if np.abs(np.array(s) - np.array(seed)).max() > 0.7:
            print("   branch jump; abort"); sys.exit(1)
        seed = s; r.move(s, 2.0, retries=2)
        r.report(f"translate {k}/3")
        print("   fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))
elif mode == "rotate":
    p, qt, _ = r.tcp(); seed = r.arm_q()
    q0 = Rot.from_quat(qt); q1 = Rot.from_quat(QB)
    rel = q1 * q0.inv(); rv = rel.as_rotvec(); n = 6
    for i in range(1, n + 1):
        qi = (Rot.from_rotvec(rv * i / n) * q0).as_quat()
        s = r.ik(p, qi, seed=seed); assert s, f"IK rot {i}"
        seed = s; r.move(s, 2.0, retries=2)
        r.report(f"rot {i}/{n}")
        print("   fingers", np.round(r.fingers(), 4), "wrench", r.wrench().round(2))
elif mode == "place":
    X, Y = float(sys.argv[2]), float(sys.argv[3])
    txy = np.array([X, Y]) + CUP_OFF2
    p, qt, _ = r.tcp(); seed = r.arm_q()
    base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    s = r.ik((txy[0], txy[1], 1.20), QB, seed=seed); assert s
    go(s, 6.0, "over perch"); seed = s
    z = 1.19
    while z >= 1.09:
        s = r.ik((txy[0], txy[1], z), QB, seed=seed); assert s
        seed = s; r.move(s, 1.0, retries=1)
        w = r.wrench(); dw = w - base_w
        r.report(f"z={z:.3f}")
        print("   dwrench", dw.round(2), "fingers", np.round(r.fingers(), 4))
        if abs(dw[2]) > 1.5 or np.abs(w[3:5]).max() > 0.4:
            print("   contact"); break
        z -= 0.01
    r.gripper(0.04)
    print("released; fingers", np.round(r.fingers(), 4))
elif mode == "retreat":
    p, qt, _ = r.tcp(); seed = r.arm_q()
    for dz in (0.04, 0.08):
        s = r.ik(p + [0, 0, dz], qt, seed=seed); assert s
        seed = s; r.move(s, 1.5, retries=2); r.report(f"up {dz}")
    s = r.ik(p + [0.10, 0.10, 0.20], qt, seed=seed)
    if s: r.move(s, 3.0, retries=2); r.report("away")
