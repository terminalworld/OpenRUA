#!/usr/bin/env python3
"""Stand up a moka pot lying on the table (pre-planned, on-branch joint paths).
Usage: python3 -u standup2.py <bx> <by> <kx> <ky> <z_axis> [d_grasp=0.068]
  (bx,by): world xy of the base face centre; (kx,ky): xy of the lid knob (axis direction);
  z_axis: height of the pot axis; d_grasp: grasp distance from the base (neck ~0.068).
Top-down grasp with fingers closing perpendicular to the axis, lift, rotate the hand about the
grasp point until the lid points up, lower until the base is ~3 mm above the table, release.
"""
import sys
import numpy as np
from geom import R_PLACE, Ry, Rz
from rob import Robot, log

bx, by, kx, ky, gz = map(float, sys.argv[1:6])
d_grasp = float(sys.argv[6]) if len(sys.argv) > 6 else 0.068
u = np.array([kx - bx, ky - by]); u /= np.linalg.norm(u)
phi = np.arctan2(u[1], u[0])
gx, gy = bx + d_grasp * u[0], by + d_grasp * u[1]
TABLE_Z = 0.90
TIP = 0.1034
R_G0 = Rz(phi) @ R_PLACE                        # hand z down, fingers close perpendicular to axis
R_F0 = Rz(phi) @ Ry(np.radians(-90)) @ R_PLACE  # hand z -> +u (lid side), pot upright
log(f"axis dir {np.round(u,3)} phi {np.degrees(phi):.1f} deg; grasp at {np.round((gx,gy,gz),3)}")

r = Robot("standup2")
q_now = r.arm_q()

G = np.array([gx, gy, gz])
G_high = G + [0, 0, 0.18]
G_down = np.array([gx, gy, TABLE_Z + d_grasp + 0.003])
FLIP = np.diag([-1.0, -1.0, 1.0])
seeds = [q_now, np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]),
         np.array([-0.5, 0.3, 0, -1.6, 0, 1.9, 0.0]), np.array([-0.8, 0.5, 0.3, -1.8, 0, 2.2, 0.5])]


def plan(fl, phi2):
    """fl: hand flip about its z; phi2: azimuth the pot axis is yawed to (about the vertical
    through the grasp point) before the righting rotation, so the arm ends in a comfortable pose."""
    R_G, R_F = R_G0 @ fl, R_F0 @ fl
    u2 = np.array([np.cos(phi2), np.sin(phi2)])
    R_G2 = Rz(phi2) @ R_PLACE @ fl
    R_F2 = Rz(phi2) @ Ry(np.radians(-90)) @ R_PLACE @ fl
    p_hover = G + [0, 0, 0.16] - TIP * R_G[:, 2]
    p_grasp = G - TIP * R_G[:, 2]
    p_high = G_high - TIP * R_G[:, 2]
    p_down = G_down - TIP * R_F2[:, 2]
    p_retreat = p_down + np.array([-0.06 * u2[0], -0.06 * u2[1], 0.12])
    tag = f"flip={fl[0,0]:+.0f} phi2={np.degrees(phi2):.0f}"
    for s in seeds:
        q_h = r.ik(p_hover, R_G, s)
        if q_h is not None and r.within_limits(q_h):
            break
    else:
        log(f"{tag}: no hover IK"); return None
    segs = {}
    steps = [("descend", lambda q: r.plan_line(q, p_grasp, R_G)),
             ("lift", lambda q: r.plan_line(q, p_high, R_G)),
             ("yaw", lambda q: r.plan_rot(q, G_high, R_G2)),
             ("rot", lambda q: r.plan_rot(q, G_high, R_F2)),
             ("down", lambda q: r.plan_line(q, p_down, R_F2)),
             ("retreat", lambda q: r.plan_line(q, p_retreat, R_F2))]
    q = q_h
    for name, fn in steps:
        seg = fn(q)
        if seg is None:
            log(f"{tag}: segment {name} infeasible"); return None
        segs[name] = seg
        q = seg[-1]
    travel = sum(np.abs(np.diff(np.array([q_h] + segs["yaw"] + segs["rot"]), axis=0)).max(1).sum() for _ in [0])
    log(f"{tag} feasible; hover q {np.round(q_h,2)}; down q {np.round(segs['down'][-1],2)}; travel {travel:.2f}")
    return dict(R_G=R_G, R_G2=R_G2, R_F2=R_F2, q_h=q_h, p_grasp=p_grasp, p_high=p_high,
                p_down=p_down, p_retreat=p_retreat, travel=travel)


plans = []
for phi2_deg in (-60, -90, -30, -120, 0, -150):
    for fl in (np.eye(3), FLIP):
        pl = plan(fl, np.radians(phi2_deg))
        if pl is not None:
            plans.append(pl)
    if plans:
        break
if not plans:
    log("no feasible plan; abort"); sys.exit(1)
P = min(plans, key=lambda p: p["travel"])
log(f"chosen plan with travel {P['travel']:.2f}")

r.gripper(0.04)
assert r.move_path([P["q_h"]], 4.0), "hover failed"
assert r.move_path(r.plan_line(r.arm_q(), P["p_grasp"], P["R_G"]), 5.0), "descend failed"
f = r.gripper(0.0)
gap = f[0] - f[1]
log(f"gap after close {gap:.4f}")
if gap < 0.04:
    log("GRASP FAILED (gap too small for a pot body)")
    r.gripper(0.04)
    r.move_path(r.plan_line(r.arm_q(), P["p_grasp"] + [0, 0, 0.16], P["R_G"]), 4.0)
    sys.exit(2)
assert r.move_path(r.plan_line(r.arm_q(), P["p_high"], P["R_G"]), 4.0), "lift failed"
log("yawing pot")
yaw = r.plan_rot(r.arm_q(), G_high, P["R_G2"])
assert yaw is not None, "yaw plan failed live"
assert r.move_path(yaw, 5.0), "yaw failed"
log("rotating pot upright")
rot = r.plan_rot(r.arm_q(), G_high, P["R_F2"])
assert rot is not None, "rotation plan failed live"
ok = r.move_path(rot, 8.0)
log(f"rotate ok={ok} fingers {r.fingers()}")
down = r.plan_line(r.arm_q(), P["p_down"], P["R_F2"])
assert down is not None, "down plan failed"
r.move_path(down, 5.0)
log(f"fingers before release {r.fingers()}")
r.gripper(0.04)
ret = r.plan_line(r.arm_q(), P["p_retreat"], P["R_F2"])
if ret is not None:
    r.move_path(ret, 3.0)
log("STANDUP DONE")
