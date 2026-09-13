#!/usr/bin/env python3
"""Third grasp: cup stands in slot C tilted ~38 deg toward +x (rim up-front), handle at -y.  Pinch the rim wall 30 deg
from the top toward +y (opposite the handle), lift, rotate so the axis is along -y with the handle pointing DOWN (keel into
the B channel), lower into B until contact, release.

Modes: plan | approach | close | lift | rotate | place X Y | retreat

Old docstring: Second grasp: cup lies on the caddy's B/C divider, rim toward +x.  Pinch the rim wall on the +y side just
below horizontal (below the handle), approaching THETA deg from the cup axis toward the up-tangent so the hand is a
horizontal bar in front of the caddy and the wrist stays high.  Lift, yaw the cup by PSI about z (rim -> -y,
handle -> +x), lower it into the B channel until contact, release.

Modes: plan | approach | lift | rotate | place X Y | retreat
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
RIM_C = np.array([-0.314, -0.119, 1.055])
c = unit([0.61, 0.0, 0.79])                          # cup axis bottom -> rim (tilted ~38 deg toward +x)
U = unit(np.array([0, 0, 1.0]) - c[2] * c)          # up, perpendicular to c
Y = unit(np.array([0, 1.0, 0]) - c[1] * c)          # +y, perpendicular to c
ROLL, THETA = np.radians(30.0), np.radians(-20.0)   # pinch 30 deg from top toward +y (handle is at -y); approach tilted 20 deg from above
INSIDE, WALL_R = 0.018, 0.0475
ROT_P = np.array([-0.36, -0.13, 1.23])              # where the cup is re-oriented
W = unit(np.sin(ROLL) * Y + np.cos(ROLL) * U)       # radial direction of pinch point (slide axis)
T = unit(np.cross(W, c))
if T[2] < 0: T = -T                                 # tangent going up
A = unit(np.cos(THETA) * (-c) - np.sin(THETA) * T)  # approach: into the cup, coming from above
SL = W
TCP = RIM_C - INSIDE * c + WALL_R * W
CENTRE = RIM_C - 0.055 * c
QA = quat(A, SL)
H0 = np.array([0, -1.0, 0]); A1 = np.array([0, -1.0, 0]); H1 = np.array([0, 0, -1.0])   # handle dir now / axis & handle after
F0 = np.stack([c, H0, np.cross(c, H0)], 1); F1 = np.stack([A1, H1, np.cross(A1, H1)], 1)
R_C = Rot.from_matrix(F1 @ F0.T)                    # cup re-orientation: axis -> -y, handle -> down
print("re-orientation angle", np.degrees(R_C.magnitude()).round(1), "W after", R_C.apply(W).round(3))
QB = (R_C * Rot.from_quat(QA)).as_quat()
OFF2 = R_C.apply(TCP - CENTRE)                      # TCP - cup centre after re-orientation
BACK, STEP = 0.10, 0.01
TARGET = np.array([-0.425, -0.135])                 # cup centre xy in the B channel

r = Robot("grasp12")

def go(q, secs, tag, retries=3, tol=0.02):
    r.move(q, secs, retries=retries)
    err = np.abs(np.array(r.arm_q()) - np.array(q)).max()
    p = r.report(tag)
    print(f"   joint err after move {err:.4f}   wrench {r.wrench().round(1)}")
    if err > tol:
        print("   !!! move did not reach goal"); sys.exit(1)
    return p

def chain(seed, steps, label):
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

def rot_steps(p, q_from, q_to, n=8):
    q0 = Rot.from_quat(q_from); rv = (Rot.from_quat(q_to) * q0.inv()).as_rotvec()
    return [(p, (Rot.from_rotvec(rv * i / n) * q0).as_quat()) for i in range(1, n + 1)]

def lift_steps(p, qt):
    return [(p + [0, 0, 0.04], qt), (p + [0, 0, 0.08], qt), (p + [0, 0, 0.12], qt)] + \
           [(p + [0, 0, 0.12] + (ROT_P - p - [0, 0, 0.12]) * k / 2, qt) for k in (1, 2)]

r.report("start")
print("A", A.round(3), "SL", SL.round(3), "TCP", TCP.round(4), "centre", CENTRE.round(4),
      "| QB hand z", Rot.from_quat(QB).apply([0, 0, 1]).round(3), "OFF2", OFF2.round(4))
if mode == "plan":
    qg, sols = ik_best(r, TCP, QA, n=40)
    for m0, st in sols[:8]:
        print("grasp cfg", np.round(st, 2), "margin", round(m0, 2))
        pre = chain(st, [(TCP - BACK * A, QA), (TCP - BACK * A + [0, 0, 0.12], QA)], "pre/hi")
        if pre is None: continue
        lift = chain(st, lift_steps(TCP, QA), "lift/translate")
        if lift is None: continue
        rot = chain(lift[-1], rot_steps(ROT_P, QA, QB), "rotate")
        if rot is None: continue
        top = np.array([TARGET[0] + OFF2[0], TARGET[1] + OFF2[1], ROT_P[2]])
        pl = chain(rot[-1], [(ROT_P + (top - ROT_P) * k / 3, QB) for k in (1, 2, 3)] +
                   [(top - [0, 0, dz], QB) for dz in (0.04, 0.08, 0.12, 0.15)], "to target/lower")
        if pl is not None and min(margin(q) for q in pre + lift + rot + pl) > 0.15:
            print("   >>> full chain OK for this grasp cfg; saving")
            json.dump({"qg": list(map(float, st))}, open("grasp12.json", "w"))
            break
elif mode == "approach":
    qg = json.load(open("grasp12.json"))["qg"]
    r.gripper(0.04)
    qpre = r.ik(TCP - BACK * A, QA, seed=qg); assert qpre
    qhi = r.ik(TCP - BACK * A + [0, 0, 0.12], QA, seed=qpre); assert qhi
    print("grasp", np.round(qg, 2), "pre", np.round(qpre, 2), "hi", np.round(qhi, 2))
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
elif mode == "close":
    f = r.gripper(0.0)
    print("after close: fingers", np.round(f, 4), "gap", round(f[0] - f[1], 4))
elif mode == "lift":
    w0 = r.wrench(); p, qt, _ = r.tcp(); seed = r.arm_q()
    sols = chain(seed, lift_steps(p, qt), "lift/translate"); assert sols
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
    X, Y2 = float(sys.argv[2]), float(sys.argv[3])
    txy = np.array([X, Y2]) + OFF2[:2]
    p, qt, _ = r.tcp(); seed = r.arm_q()
    base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    top = np.array([txy[0], txy[1], p[2]])
    sols = chain(seed, [(p + (top - p) * k / 3, QB) for k in (1, 2, 3)], "to target"); assert sols
    for i, s in enumerate(sols):
        go(s, 3.0, f"to target {i + 1}")
    seed = sols[-1]; z = top[2] - 0.02
    ZMIN = 1.045 + OFF2[2]                          # cup centre no lower than 1.045 (settled wedge height 1.036)
    while z >= ZMIN - 1e-6:
        s = r.ik((txy[0], txy[1], z), QB, seed=seed); assert s
        seed = s; r.move(s, 1.0, retries=1)
        w = r.wrench(); dw = w - base_w
        r.report(f"z={z:.3f} (cup centre z={z - OFF2[2]:.3f})")
        print("   dwrench", dw.round(2), "fingers", np.round(r.fingers(), 4))
        if np.abs(dw[:3]).max() > 4.0 or np.abs(dw[3:5]).max() > 1.0:
            print("   firm contact, stop"); break
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
