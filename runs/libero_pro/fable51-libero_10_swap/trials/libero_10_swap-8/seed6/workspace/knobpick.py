#!/usr/bin/env python3
"""Pinch a moka pot's lid knob from above and lift it (the pot then hangs upright).
Usage: python3 -u knobpick.py <kx> <ky> <kz_tips> <phi_deg> [lift=0.22]
  (kx,ky): world xy of the knob cap centre; kz_tips: fingertip height for the pinch;
  phi_deg: azimuth of the pot axis (fingers close perpendicular to it; irrelevant for an upright pot).
"""
import sys
import numpy as np
from geom import R_PLACE, Rz
from rob import Robot, log

kx, ky, kz, phi = map(float, sys.argv[1:5])
lift = float(sys.argv[5]) if len(sys.argv) > 5 else 0.22
TIP = 0.1034
R = Rz(np.radians(phi)) @ R_PLACE
r = Robot("knobpick")
FLIP = np.diag([-1.0, -1.0, 1.0])
p_grasp = np.array([kx, ky, kz]) - TIP * R[:, 2]
p_hover = p_grasp + [0, 0, 0.15]
p_lift = p_grasp + [0, 0, lift]

best = None
for fl in (np.eye(3), FLIP):
    for s in (r.arm_q(), np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]),
              np.array([-0.5, 0.3, 0, -1.6, 0, 1.9, 0.0])):
        q = r.ik(p_hover, R @ fl, s)
        if q is None or not r.within_limits(q):
            continue
        d = r.plan_line(q, p_grasp, R @ fl)
        if d is None:
            continue
        u = r.plan_line(d[-1], p_lift, R @ fl)
        if u is None:
            continue
        travel = np.abs(q - r.arm_q()).max()
        if best is None or travel < best[0]:
            best = (travel, q, R @ fl)
assert best is not None, "no feasible hover/descent"
_, q_h, Rg = best
log(f"hover q {np.round(q_h,2)} fingers dir {np.round(Rg[:,1],3)}")
r.gripper(0.04)
assert r.move_path([q_h], 4.0), "hover failed"
assert r.move_path(r.plan_line(r.arm_q(), p_grasp, Rg), 5.0), "descend failed"
p, _ = r.fk()
log(f"tips at {np.round(p + TIP * Rg[:,2], 4)}")
f = r.gripper(0.0)
gap = f[0] - f[1]
log(f"gap after close {gap:.4f}")
if gap < 0.004:
    log("PINCH FAILED (closed on air)")
    r.gripper(0.04)
    r.move_path(r.plan_line(r.arm_q(), p_hover, Rg), 4.0)
    sys.exit(2)
assert r.move_path(r.plan_line(r.arm_q(), p_lift, Rg), 6.0), "lift failed"
f = r.fingers()
log(f"after lift gap {f[0]-f[1]:.4f}")
log("KNOBPICK DONE")
