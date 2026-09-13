#!/usr/bin/env python3
"""Pick a LYING moka pot by its lid knob, right it (rotate about the pinch axis), carry it to the
stove and set it down. Everything is pre-planned on one IK branch before touching the pot.
Usage: python3 -u pot1.py <kx> <ky> <kz_axis> <ax> <ay> <tx> <ty> <surface_z> [phase]
  (kx,ky,kz_axis): knob cap centre; (ax,ay): horizontal direction knob -> pot base;
  (tx,ty): where the pot base should stand on the stove. phase: 'plan' (default) / 'go'.
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from geom import R_PLACE, Rz
from rob import Robot, log

kx, ky, kz, ax, ay, tx, ty, PLATE_Z = map(float, sys.argv[1:9])
GO = len(sys.argv) > 9 and sys.argv[9] == "go"
TIP, HZ = 0.1034, 0.093            # fingertip / pad-centre distance along hand z
LIFT = 0.21
KNOB_ABOVE_BASE = 0.150
CARRY_Z = 1.16                    # knob height while carrying (pot base ~1.01, clears everything)
a = np.array([ax, ay, 0.0]); a /= np.linalg.norm(a)
phi = np.arctan2(a[1], a[0])
FLIP = np.diag([-1.0, -1.0, 1.0])
r = Robot("pot1")
q_now = r.arm_q()
K = np.array([kx, ky, kz])                       # knob cap centre (pinch point)
K_lift = K + [0, 0, LIFT]
BASE = np.array([-0.66, 0.0])


def azim(K):
    """Azimuth of the hand direction pointing radially away from the robot base at K."""
    return np.arctan2(K[1] - BASE[1], K[0] - BASE[0])


def plan_carry(q0, K0, K1, R0, step=0.03):
    """Carry the pinch point from K0 to K1 keeping the hand horizontal and pointing radially away
    from the base (the pot stays vertical since the hand only yaws about the vertical)."""
    n = max(1, int(np.ceil(np.linalg.norm(K1 - K0) / step)))
    path, q = [], np.array(q0)
    for i in range(1, n + 1):
        Ki = K0 + (K1 - K0) * i / n
        Ri = Rz(azim(Ki) - azim(K0)) @ R0
        q = r.ik_local(Ki - HZ * Ri[:, 2], Ri, q)
        if q is None or not r.within_limits(q):
            log(f"plan_carry: fail at {i}/{n}" + ("" if q is None else f" (limits) {np.round(q,2)}"))
            return None, None
        path.append(q)
    return path, Ri


def right_R(R, a_now):
    """Orientation after rotating about the finger axis so a_now -> -z."""
    n = R[:, 1]
    aa = a_now - n * (a_now @ n); aa /= np.linalg.norm(aa)
    b = np.array([0, 0, -1.0]); b = b - n * (b @ n); b /= np.linalg.norm(b)
    ang = np.arctan2(np.cross(aa, b) @ n, aa @ b)
    return Rot.from_rotvec(n * ang).as_matrix() @ R


def hover_solutions(fl):
    R_G = Rz(phi) @ R_PLACE @ fl
    p_hover = K - HZ * R_G[:, 2] + [0, 0, 0.15]
    sols = []
    for s in (q_now, np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]),
              np.array([-0.5, 0.3, 0, -1.6, 0, 1.9, 0.0]), np.array([-0.8, 0.5, 0.3, -1.8, 0, 2.2, 0.5]),
              np.array([0.3, 0.6, -0.3, -1.6, 0.3, 2.0, -1.5]), np.array([-0.3, 0.9, 0.6, -1.1, -0.5, 1.9, -2.5])):
        for _ in range(2):
            q = r.ik(p_hover, R_G, s)
            if q is not None and r.within_limits(q) and all(np.abs(q - o).max() > 0.05 for o in sols):
                sols.append(q)
    log(f"flip={fl[0,0]:+.0f}: {len(sols)} hover solutions")
    return sols


def plan(fl, yaw_deg, rot_first=False, q_h=None):
    tag = f"flip={fl[0,0]:+.0f} yaw={yaw_deg:+.0f} rot_first={rot_first} j7h={q_h[6]:+.2f}"
    R_G = Rz(phi) @ R_PLACE @ fl                  # z down, fingers perpendicular to the pot axis
    p_grasp = K - HZ * R_G[:, 2]
    p_hover = p_grasp + [0, 0, 0.15]
    p_lift = p_grasp + [0, 0, LIFT]
    R_Y = Rz(np.radians(yaw_deg)) @ R_G
    a_y = Rz(np.radians(yaw_deg)) @ a
    R_F = right_R(R_Y, a_y)                       # hand horizontal, z = -a_y
    R_F0 = right_R(R_G, a)                        # righted before yawing (rot_first)
    segs, q = {}, q_h
    K_carry = np.array([tx, ty, CARRY_Z])
    K_down = np.array([tx, ty, PLATE_Z + KNOB_ABOVE_BASE + 0.002])
    K_up = np.array([K_lift[0], K_lift[1], CARRY_Z])
    R_T = Rz(azim(K_carry) - azim(K_lift)) @ R_F     # hand orientation at the stove
    steps = [("descend", lambda q: r.plan_line(q, p_grasp, R_G)),
             ("lift", lambda q: r.plan_line(q, p_lift, R_G)),
             ("rot", lambda q: r.plan_rot(q, K_lift, R_F0, step_deg=6)),
             ("yaw", lambda q: r.plan_rot(q, K_lift, R_F, step_deg=6)),
             ("up", lambda q: r.plan_line(q, K_up - HZ * R_F[:, 2], R_F, step=0.03)),
             ("carry", lambda q: plan_carry(q, K_up, K_carry, R_F)[0]),
             ("down", lambda q: r.plan_line(q, K_down - HZ * R_T[:, 2], R_T)),
             ("retreat", lambda q: r.plan_line(q, K_down - HZ * R_T[:, 2] - 0.08 * R_T[:, 2] + [0, 0, 0.06], R_T))]
    if not rot_first:
        steps[2:4] = [("yaw", lambda q: r.plan_rot(q, K_lift, R_Y, step_deg=8)),
                      ("rot", lambda q: r.plan_rot(q, K_lift, R_F, step_deg=6))]
    for name, fn in steps:
        seg = fn(q)
        if seg is None:
            log(f"{tag}: segment {name} infeasible"); return None
        segs[name] = seg
        q = seg[-1]
    allq = np.array([q_now, q_h] + sum((segs[n] for n, _ in steps), []))
    travel = np.abs(np.diff(allq, axis=0)).max(1).sum()
    log(f"{tag}: feasible, travel {travel:.2f}; hover q {np.round(q_h,2)}; final hand z {np.round(R_F[:,2],2)}")
    return dict(tag=tag, q_h=q_h, segs=segs, travel=travel, R_G=R_G, R_Y=R_Y, R_F=R_F, R_F0=R_F0, R_T=R_T,
                K_up=K_up,
                rot_first=rot_first, order=[n for n, _ in steps],
                p_grasp=p_grasp, p_hover=p_hover, p_lift=p_lift, K_carry=K_carry, K_down=K_down)


plans = []
HOV = {+1: hover_solutions(np.eye(3)), -1: hover_solutions(FLIP)}
yaw_radial = np.degrees(azim(K) + np.pi - phi)     # hand ends pointing radially away from the base
for yaw_deg in (yaw_radial, yaw_radial - 30, yaw_radial + 30, np.degrees(np.pi - phi), yaw_radial - 60, yaw_radial + 60):
    yaw_deg = (yaw_deg + 180) % 360 - 180
    for fl in (np.eye(3), FLIP):
        for q_h in HOV[int(fl[0, 0])]:
            for rf in (False, True):
                pl = plan(fl, yaw_deg, rf, q_h)
                if pl is not None:
                    plans.append(pl)
    if plans:
        break
if not plans:
    log("NO FEASIBLE PLAN"); sys.exit(1)
P = min(plans, key=lambda p: p["travel"])
log(f"chosen {P['tag']}")
if not GO:
    sys.exit(0)


def chunks(path, n, secs, what):
    for i in range(0, len(path), n):
        ok = r.move_path(path[i:i + n], secs)
        f = r.fingers()
        log(f"{what} chunk ok={ok} gap {f[0]-f[1]:.4f}")
        if not ok or f[0] - f[1] < 0.004:
            log(f"ABORT during {what}"); sys.exit(1)


r.gripper(0.04)
assert r.move_path([P["q_h"]], 5.0), "hover failed"
assert r.move_path(r.plan_line(r.arm_q(), P["p_grasp"], P["R_G"]), 5.0), "descend failed"
p, _ = r.fk()
log(f"pads at {np.round(p + HZ * P['R_G'][:,2], 4)} (knob {np.round(K,4)})")
f = r.gripper(0.0)
gap = f[0] - f[1]
log(f"gap after close {gap:.4f}")
if gap < 0.004:
    log("PINCH FAILED (closed on air)")
    r.gripper(0.04)
    r.move_path(r.plan_line(r.arm_q(), P["p_hover"], P["R_G"]), 4.0)
    sys.exit(2)
assert r.move_path(r.plan_line(r.arm_q(), P["p_lift"], P["R_G"]), 5.0), "lift failed"
# re-plan the remaining segments from the actual configuration (same branch)
if P["rot_first"]:
    seq = [("rot", P["R_F0"]), ("yaw", P["R_F"])]
else:
    seq = [("yaw", P["R_Y"]), ("rot", P["R_F"])]
for name, RR in seq:
    seg = r.plan_rot(r.arm_q(), K_lift, RR, step_deg=6); assert seg is not None, f"{name} replan"
    chunks(seg, 4, 4.0, name)
p, qu = r.fk()
log(f"righted: hand {np.round(p,4)} z {np.round(Rot.from_quat(qu).as_matrix()[:,2],3)} fingers {r.fingers()}")
log("RIGHTED -- pausing here for a visual check")
