#!/usr/bin/env python3
"""Open the microwave door: pinch the handle bar (fingers along the door's
x axis, hand tilted THETA below horizontal) and swing it about the hinge,
rotating the hand with the door."""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from ctl import Robot, quat_from_axes, log

H = np.array([-0.262, -0.35])           # hinge axis (world x,y)
B0 = np.array([-0.015, -0.399, 1.062])   # pinch near the top so the housing clears the bar top   # handle bar centre, door closed
THETA = np.radians(float(sys.argv[3]) if len(sys.argv) > 3 else 80.0)
A0 = np.array([0.0, np.cos(THETA), -np.sin(THETA)])   # approach: +y and down
FX0 = np.array([1.0, 0.0, 0.0])                       # finger closing axis
PAD = 0.012        # tip goes this far past bar centre so pads sit on the bar
PHI_MAX = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0


def pose(phi_deg, back=0.0, dz=0.0):
    Rz = Rot.from_euler("z", -phi_deg, degrees=True)
    a, fx = Rz.apply(A0), Rz.apply(FX0)
    rel = Rz.apply([B0[0] - H[0], B0[1] - H[1], 0.0])
    bar = np.array([H[0] + rel[0], H[1] + rel[1], B0[2] + dz])
    return bar + (PAD - back) * a, quat_from_axes(a, fx), a


r = Robot("door")
dry = len(sys.argv) > 1 and sys.argv[1] == "dry"

# ---- plan everything first ----
seed = r.joints()
plan = []
pre = [(0.0, 0.08, 0.12), (0.0, 0.08, 0.0), (0.0, 0.05, 0.0), (0.0, 0.025, 0.0), (0.0, 0.0, 0.0)]
for phi, back, dz in pre:
    tip, q, a = pose(phi, back, dz)
    sol = r.ik(tip, q, seed=seed, at_tcp=True)
    log(f"pre phi={phi} back={back} dz={dz} tip={np.round(tip,3)} -> {None if sol is None else np.round(sol,2)}")
    if sol is None:
        sys.exit("IK failed in approach")
    plan.append(sol); seed = sol
swing = []
phis = np.arange(10.0, PHI_MAX + 0.1, 10.0)
for phi in phis:
    tip, q, a = pose(phi)
    sol = r.ik(tip, q, seed=seed, at_tcp=True)
    jump = None if sol is None else np.abs(np.array(sol) - np.array(seed)).max()
    log(f"swing phi={phi} tip={np.round(tip,3)} -> {None if sol is None else np.round(sol,2)} jump={jump}")
    if sol is None:
        log("stopping swing plan here"); phis = phis[:len(swing)]; break
    swing.append(sol); seed = sol
if dry:
    sys.exit(0)

# ---- execute ----
log("== lift straight up first")
hx, hq = r.hand_pose()
lift = r.ik(hx + [0, 0, 0.12], hq)
if lift is None:
    sys.exit("lift IK failed")
r.move_joints([lift], [4.0])
log("== move to high pre-pose")
r.move_joints([plan[0]], [max(3.0, np.abs(np.array(plan[0]) - np.array(r.joints())).max() / 0.4)])
log("== open gripper"); r.gripper(0.04)
log("== descend to pre-grasp"); r.move_joints([plan[1]], [4.0])
log("== approach"); r.move_joints(plan[2:], [2.0, 4.0, 6.0])
log("hand", np.round(r.hand_pose()[0], 3))
log("== close gripper"); f = r.gripper(0.0)
if f[0] < 0.007:
    log("!! fingers closed fully - missed the bar"); sys.exit(1)
log("== swing")
times = [3.0 + 2.0 * i for i in range(len(swing))]
r.move_joints(swing, times, hold=3.0)
log("== release"); r.gripper(0.04)
tip, q, a = pose(phis[-1], back=0.10)
sol = r.ik(tip, q, at_tcp=True)
if sol is not None:
    r.move_joints([sol], [3.0])
tip, q, a = pose(phis[-1], back=0.08, dz=0.10)
sol = r.ik(tip, q, at_tcp=True)
if sol is not None:
    r.move_joints([sol], [3.0])
log("done; fingers", r.fingers())
