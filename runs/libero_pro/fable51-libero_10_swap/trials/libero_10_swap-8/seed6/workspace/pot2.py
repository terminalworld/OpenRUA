#!/usr/bin/env python3
"""Pick an UPRIGHT moka pot by its lid knob from above, carry it, set it down.
Usage: python3 -u pot2.py <kx> <ky> <kz_cap_centre> <tx> <ty> <surface_z> <handx_azim_deg>
"""
import sys
import numpy as np
from geom import R_PLACE, Rz
from rob import Robot, log

kx, ky, kz, tx, ty, SZ, psi = map(float, sys.argv[1:8])
HZ = 0.093
LIFT = 0.12
R = Rz(np.radians(psi)) @ R_PLACE
r = Robot("pot2")
q_now = r.arm_q()
K = np.array([kx, ky, kz])
KAB = kz - 0.90                                   # pinch point height above the pot base (table at 0.90)
K_lift = K + [0, 0, LIFT]
K_carry = np.array([tx, ty, kz + LIFT])
K_down = np.array([tx, ty, SZ + KAB + 0.002])
hp = lambda Kp: Kp - HZ * R[:, 2]                 # hand origin for a given pad-centre point

# hover solutions, pre-plan the whole chain on one branch
plans = []
for s in (q_now, np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785]), np.array([-0.5, 0.3, 0, -1.6, 0, 1.9, 0.0]),
          np.array([0.3, 0.6, -0.3, -1.6, 0.3, 2.0, -1.5]), np.array([-0.3, 0.9, 0.6, -1.1, -0.5, 1.9, -2.5])):
    q_h = r.ik(hp(K_lift), R, s)
    if q_h is None or not r.within_limits(q_h) or any(np.abs(q_h - p["q_h"]).max() < 0.05 for p in plans):
        continue
    segs, q, ok = {}, q_h, True
    for name, fn in [("descend", lambda q: r.plan_line(q, hp(K), R)),
                     ("lift", lambda q: r.plan_line(q, hp(K_lift), R)),
                     ("carry", lambda q: r.plan_line(q, hp(K_carry), R, step=0.03)),
                     ("down", lambda q: r.plan_line(q, hp(K_down), R)),
                     ("retreat", lambda q: r.plan_line(q, hp(K_down) + [0, 0, 0.06], R))]:
        seg = fn(q)
        if seg is None:
            log(f"hover j7={q_h[6]:+.2f}: {name} infeasible"); ok = False; break
        segs[name] = seg; q = seg[-1]
    if ok:
        travel = np.abs(np.diff(np.array([q_now, q_h] + sum(segs.values(), [])), axis=0)).max(1).sum()
        log(f"hover q {np.round(q_h,2)} feasible, travel {travel:.2f}")
        plans.append(dict(q_h=q_h, travel=travel))
assert plans, "no feasible plan"
P = min(plans, key=lambda p: p["travel"])

r.gripper(0.04)
assert r.move_path([P["q_h"]], 5.0), "hover failed"
assert r.move_path(r.plan_line(r.arm_q(), hp(K), R), 5.0), "descend failed"
p, _ = r.fk()
log(f"pads at {np.round(p + HZ * R[:,2], 4)} (target {np.round(K,4)})")
f = r.gripper(0.0)
gap = f[0] - f[1]
log(f"gap after close {gap:.4f}")
if gap < 0.004:
    log("PINCH FAILED"); r.gripper(0.04)
    r.move_path(r.plan_line(r.arm_q(), hp(K_lift), R), 4.0); sys.exit(2)
for name, target in [("lift", K_lift), ("carry", K_carry)]:
    seg = r.plan_line(r.arm_q(), hp(target), R, step=0.03); assert seg is not None, f"{name} replan"
    for i in range(0, len(seg), 6):
        ok = r.move_path(seg[i:i + 6], 4.0)
        f = r.fingers()
        log(f"{name} chunk ok={ok} gap {f[0]-f[1]:.4f}")
        if not ok or f[0] - f[1] < 0.004:
            log(f"ABORT during {name}"); sys.exit(1)
seg = r.plan_line(r.arm_q(), hp(K_down), R); assert seg is not None, "down replan"
ok = r.move_path(seg, 4.0)
p, _ = r.fk()
log(f"down ok={ok}; pads at {np.round(p + HZ * R[:,2], 4)} (target {np.round(K_down,4)}) fingers {r.fingers()}")
r.gripper(0.04)
seg = r.plan_line(r.arm_q(), hp(K_down) + [0, 0, 0.06], R); assert seg is not None, "retreat replan"
log(f"retreat ok={r.move_path(seg, 4.0)}")
log("POT2 DONE")
