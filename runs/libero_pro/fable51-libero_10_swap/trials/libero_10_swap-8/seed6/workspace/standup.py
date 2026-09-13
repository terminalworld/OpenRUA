#!/usr/bin/env python3
"""Stand up a moka pot lying on the table.
Usage: python3 -u standup.py <bx> <by> <kx> <ky> <z_axis> [d_grasp=0.05]
  (bx,by): world xy of the base face centre; (kx,ky): xy of the lid knob (defines the axis
  direction); z_axis: height of the pot axis; d_grasp: grasp distance from the base.
Top-down grasp with fingers closing perpendicular to the axis, lift, rotate -90deg about the
horizontal axis perpendicular to the pot so the lid points up, lower until the base is ~3 mm
above the table, release, retreat.
"""
import sys
import numpy as np
from geom import R_PLACE, Ry, Rz
from rob import Robot, log

bx, by, kx, ky, gz = map(float, sys.argv[1:6])
d_grasp = float(sys.argv[6]) if len(sys.argv) > 6 else 0.05
u = np.array([kx - bx, ky - by]); u /= np.linalg.norm(u)
phi = np.arctan2(u[1], u[0])
gx, gy = bx + d_grasp * u[0], by + d_grasp * u[1]
TABLE_Z = 0.90
TIP = 0.1034
R_G = Rz(phi) @ R_PLACE                        # hand z down, fingers close perpendicular to axis
R_F = Rz(phi) @ Ry(np.radians(-90)) @ R_PLACE  # hand z -> along axis dir (lid side), pot upright
h_above_base = d_grasp
log(f"axis dir {np.round(u,3)} phi {np.degrees(phi):.1f} deg; grasp at {np.round((gx,gy,gz),3)}")
log(f"R_F z-axis {np.round(R_F[:,2],3)} x-axis {np.round(R_F[:,0],3)}")

r = Robot("standup")
q0 = r.arm_q()

# poses (hand frame): fingertips at the axis point for the grasp
tips_hover = np.array([gx, gy, gz + 0.16])
tips_grasp = np.array([gx, gy, gz])
p_hover = tips_hover - TIP * R_G[:, 2]
p_grasp = tips_grasp - TIP * R_G[:, 2]
G_high = np.array([gx, gy, gz + 0.18])          # axis point carried high
p_high_v = G_high - TIP * R_G[:, 2]              # still vertical
p_high_h = G_high - TIP * R_F[:, 2]              # rotated: hand behind (-x) the pot
G_down = np.array([gx, gy, TABLE_Z + h_above_base + 0.003])
p_down = G_down - TIP * R_F[:, 2]
p_retreat = p_down + np.array([-0.06*u[0], -0.06*u[1], 0.12])

# feasibility check before moving anything; the pot rotation must be exactly Ry(-90) applied
# to the ACTUAL grasp orientation, so pick one flip (about hand z) and keep it for every pose.
FLIP = np.diag([-1.0, -1.0, 1.0])
chain = None
for fl in (np.eye(3), FLIP):
    RG, RF = R_G @ fl, R_F @ fl
    seed, qs, ok = q0, [], True
    plan = [("hover", p_hover, RG), ("grasp", p_grasp, RG), ("high_v", p_high_v, RG),
            ("high_h", p_high_h, RF), ("down", p_down, RF), ("retreat", p_retreat, RF)]
    for name, p, R in plan:
        q = r.ik(p, R, seed)
        if q is None:
            log(f"flip={fl[0,0]}: IK infeasible for {name} at {np.round(p,3)}")
            ok = False
            break
        log(f"flip={fl[0,0]} {name}: hand {np.round(p,3)} ok, joint travel {np.round(np.abs(q-seed).max(),2)}")
        seed = q
    if ok:
        chain = (RG, RF)
        break
if chain is None:
    log("no feasible chain; abort")
    sys.exit(1)
R_G, R_F = chain

r.gripper(0.04)
assert r.move_pose(p_hover, R_G, 4.0, flip_ok=False), "hover failed"
assert r.move_line(p_grasp, R_G, 5.0), "descend failed"
f = r.gripper(0.0)
gap = f[0] - f[1]
log(f"gap after close {gap:.4f}")
if gap < 0.04:
    log("GRASP FAILED (gap too small for a pot body)")
    r.gripper(0.04)
    r.move_pose(p_hover, R_G, 3.0, flip_ok=False)
    sys.exit(2)
assert r.move_line(p_high_v, R_G, 4.0), "lift failed"
log("rotating pot upright")
ok = r.move_pose(p_high_h, R_F, 6.0, flip_ok=False)
log(f"rotate ok={ok} fingers {r.fingers()}")
r.move_line(p_down, R_F, 5.0, retries=0)
log(f"fingers before release {r.fingers()}")
r.gripper(0.04)
r.move_line(p_retreat, R_F, 3.0, retries=0)
log("STANDUP DONE")
