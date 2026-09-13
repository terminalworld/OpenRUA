#!/usr/bin/env python3
"""Close the microwave door: vertical pinch of the handle bar at phi=90, swing to phi=0."""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from ctl import Robot, quat_from_axes, log

H = np.array([-0.247, -0.348])   # fitted from closed (-0.015,-0.399) and open-90 (-0.299,-0.580) bar centres
B0 = np.array([-0.015, -0.399, 1.062])
A0 = np.array([0.0, 0.0, -1.0])
FX0 = np.array([1.0, 0.0, 0.0])
PAD = 0.012

def pose(phi_deg, back=0.0, dz=0.0):
    Rz = Rot.from_euler("z", -phi_deg, degrees=True)
    a, fx = Rz.apply(A0), Rz.apply(FX0)
    rel = Rz.apply([B0[0] - H[0], B0[1] - H[1], 0.0])
    bar = np.array([H[0] + rel[0], H[1] + rel[1], B0[2] + dz])
    return bar + (PAD - back) * a, quat_from_axes(a, fx), a

r = Robot("doorc")
dry = "dry" in sys.argv
seed = r.joints()
plan = []
for phi, back, dz in [(90, 0.08, 0.12), (90, 0.08, 0.0), (90, 0.05, 0.0), (90, 0.025, 0.0), (90, 0.0, 0.0)]:
    tip, q, a = pose(phi, back, dz)
    sol = r.ik(tip, q, seed=seed, at_tcp=True)
    log(f"pre back={back} dz={dz} tip={np.round(tip,3)} -> {None if sol is None else np.round(sol,2)}")
    if sol is None: sys.exit("IK failed in approach")
    plan.append(sol); seed = sol
swing = []
phis = np.arange(80.0, -0.1, -10.0)
for phi in phis:
    tip, q, a = pose(phi)
    sol = r.ik(tip, q, seed=seed, at_tcp=True)
    if sol is None: sys.exit(f"IK failed swing {phi}")
    log(f"swing phi={phi} tip={np.round(tip,3)} jump={np.abs(np.array(sol)-np.array(seed)).max():.2f}")
    swing.append(sol); seed = sol
if dry: sys.exit(0)

log("== lift straight up")
hx, hq = r.hand_pose()
if hx[2] < 1.3:
    lift = r.ik(hx + [0, 0, 0.12], hq)
    r.move_joints([lift], [4.0])
log("== open"); r.gripper(0.04)
log("== high pre-pose"); r.move_joints([plan[0]], [max(3.0, np.abs(np.array(plan[0]) - np.array(r.joints())).max() / 0.4)])
log("== descend"); r.move_joints([plan[1]], [4.0])
log("== approach"); r.move_joints(plan[2:], [2.0, 4.0, 6.0])
log("hand", np.round(r.hand_pose()[0], 3))
log("== close"); f = r.gripper(0.0)
if f[0] < 0.007: sys.exit(f"!! missed the bar {f}")
log("== swing")
r.move_joints(swing, [3.0 + 2.0 * i for i in range(len(swing))], hold=3.0)
log("hand", np.round(r.hand_pose()[0], 3), "fingers", r.fingers())
log("== release"); r.gripper(0.04)
tip, q, a = pose(0.0, back=0.12)
sol = r.ik(tip, q, at_tcp=True)
if sol is not None: r.move_joints([sol], [3.0])
log("done; hand", np.round(r.hand_pose()[0], 3))
