#!/usr/bin/env python3
"""Rim-wall pinch of the cup lying on its side, approaching obliquely (THETA deg from the cup axis toward
the wall tangent) so the wrist stays clear of the caddy; then lift, translate, upright the cup and perch it.

Modes: plan [psi] | approach | lift | rotate | place X Y | retreat
Geometry: cup axis c (bottom->rim), rim centre RIM_C.  Pinch point on the rim wall at ROLL deg from the top
(toward -y for negative), INSIDE m inside the rim.  Slide axis = radial W (pads flat on the wall).
"""
import sys, json
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot
from ikbest import ik_best, margin

def unit(v):
    v = np.array(v, float); return v / np.linalg.norm(v)

def quat(z, y):
    z = unit(z); y = unit(y); x = np.cross(y, z)
    return Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()

mode = sys.argv[1]
RIM_C = np.array([-0.219, 0.088, 0.943])
c = unit([-0.964, 0.150, 0.150])
U = unit(np.array([0, 0, 1.0]) - c[2] * c)          # up, perpendicular to c
S = unit(np.cross(U, -c))                           # side (+y)
ROLL, THETA = np.radians(-50.0), np.radians(50.0)
INSIDE, WALL_R = 0.018, 0.0475
PSI = float(sys.argv[2]) if mode == "plan" and len(sys.argv) > 2 else json.load(open("grasp10.json"))["psi"] if mode != "plan" else 0.0
ROT_P = np.array([-0.24, 0.12, 1.12])               # where the cup is uprighted
W = unit(np.sin(ROLL) * S + np.cos(ROLL) * U)       # radial direction of pinch point (slide axis)
T = unit(np.cross(W, c))                            # tangent at pinch point
if np.dot(T, S) < 0: T = -T                         # choose tangent going toward +y / up
A = unit(np.cos(THETA) * (-c) - np.sin(THETA) * T)  # approach: into cup, drifting so wrist goes up/+y
SL = W
TCP = RIM_C - INSIDE * c + WALL_R * W
QA = quat(A, SL)
ax = unit(np.cross(c, [0, 0, 1.0])); ang = np.arccos(np.clip(c[2], -1, 1))
R_UP = Rot.from_euler("z", PSI, degrees=True) * Rot.from_rotvec(ax * ang)
QB = (R_UP * Rot.from_quat(QA)).as_quat()
H = R_UP.apply(W)                                   # horizontal: TCP = cup centre + WALL_R*H when upright
CUP_OFF2 = WALL_R * H[:2]
BACK, STEP = 0.10, 0.01
PERCH = np.array([-0.458, -0.082])

r = Robot("grasp10")

def go(q, secs, tag, retries=3, tol=0.02):
    r.move(q, secs, retries=retries)
    err = np.abs(np.array(r.arm_q()) - np.array(q)).max()
    p = r.report(tag)
    print(f"   joint err after move {err:.4f}   wrench {r.wrench().round(1)}")
    if err > tol:
        print("   !!! move did not reach goal"); sys.exit(1)
    return p

def chain(seed, steps, label):
    """IK along a list of (pos, quat); returns list of solutions or None; prints margins."""
    out = []
    for i, (pos, qt) in enumerate(steps):
        s = r.ik(pos, qt, seed=seed, timeout=0.4)
        if not s:
            print(f"   {label}[{i}] IK fail"); return None
        jump = np.abs(np.array(s) - np.array(seed)).max()
        if jump > 0.8:
            print(f"   {label}[{i}] branch jump {jump:.2f}"); return None
        out.append(s); seed = s
    print(f"   {label}: min margin {min(margin(s) for s in out):.2f}")
    return out

def rot_steps(p, q_from, q_to, n=6):
    q0 = Rot.from_quat(q_from); rv = (Rot.from_quat(q_to) * q0.inv()).as_rotvec()
    return [(p, (Rot.from_rotvec(rv * i / n) * q0).as_quat()) for i in range(1, n + 1)]

r.report("start")
print("A", A.round(3), "SL", SL.round(3), "TCP", TCP.round(4), "| QB hand z", Rot.from_quat(QB).apply([0, 0, 1]).round(3),
      "H", H.round(3), "psi", PSI)
if mode == "plan":
    qg, sols = ik_best(r, TCP, QA, n=20)
    for m0, st in sols[:4]:
        print("grasp cfg", np.round(st, 2), "margin", round(m0, 2))
        pre = chain(st, [(TCP - BACK * A, QA), (TCP - BACK * A + [0, 0, 0.12], QA)], "pre/hi")
        lift = chain(st, [(TCP + [0, 0, 0.03], QA), (TCP + [0, 0, 0.06], QA)] +
                     [(TCP + [0, 0, 0.06] + (ROT_P - TCP - [0, 0, 0.06]) * k / 3, QA) for k in (1, 2, 3)], "lift/translate")
        if lift is None: continue
        rot = chain(lift[-1], rot_steps(ROT_P, QA, QB), "rotate")
        if rot is None: continue
        perch_tcp = np.array([PERCH[0] + CUP_OFF2[0], PERCH[1] + CUP_OFF2[1], 1.21])
        pl = chain(rot[-1], [(ROT_P + (perch_tcp - ROT_P) * k / 3, QB) for k in (1, 2, 3)] +
                   [(perch_tcp - [0, 0, dz], QB) for dz in (0.04, 0.08, 0.11)], "to perch/lower")
        if pl is not None:
            print("   >>> full chain OK for this grasp cfg; saving")
            json.dump({"psi": PSI, "qg": list(map(float, st))}, open("grasp10.json", "w"))
            break
elif mode == "approach":
    cfg = json.load(open("grasp10.json")); qg = cfg["qg"]
    r.gripper(0.04)
    qpre = r.ik(TCP - BACK * A, QA, seed=qg); assert qpre
    qhi = r.ik(TCP - BACK * A + [0, 0, 0.12], QA, seed=qpre); assert qhi
    print("grasp", np.round(qg, 2), "pre", np.round(qpre, 2), "hi", np.round(qhi, 2))
    p0, qt0, _ = r.tcp()
    ql = r.ik(p0 + [0, 0, max(0, 1.20 - p0[2])], qt0, seed=r.arm_q())
    if ql: go(ql, 4.0, "lift-current")
    go(qhi, 8.0, "hi", retries=4)
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
    steps = [(p + [0, 0, 0.03], qt), (p + [0, 0, 0.06], qt)] + \
            [(p + [0, 0, 0.06] + (ROT_P - p - [0, 0, 0.06]) * k / 3, qt) for k in (1, 2, 3)]
    sols = chain(seed, steps, "lift/translate"); assert sols
    for i, s in enumerate(sols):
        r.move(s, 2.0, retries=2); r.report(f"lift step {i}")
        print("   fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))
elif mode == "rotate":
    p, qt, _ = r.tcp(); seed = r.arm_q()
    sols = chain(seed, rot_steps(p, qt, QB), "rotate"); assert sols
    for i, s in enumerate(sols):
        r.move(s, 2.5, retries=2); r.report(f"rot {i + 1}")
        print("   fingers", np.round(r.fingers(), 4), "wrench", r.wrench().round(2))
elif mode == "place":
    X, Y = float(sys.argv[2]), float(sys.argv[3])
    txy = np.array([X, Y]) + CUP_OFF2
    p, qt, _ = r.tcp(); seed = r.arm_q()
    base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    top = np.array([txy[0], txy[1], max(p[2], 1.24)])   # translate high: cup bottom must clear caddy top (1.056)
    sols = chain(seed, [(p + (top - p) * k / 3, QB) for k in (1, 2, 3)], "to perch"); assert sols
    for i, s in enumerate(sols):
        go(s, 3.0, f"to perch {i + 1}")
    seed = sols[-1]; z = top[2] - 0.02
    while z >= 1.09:
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
