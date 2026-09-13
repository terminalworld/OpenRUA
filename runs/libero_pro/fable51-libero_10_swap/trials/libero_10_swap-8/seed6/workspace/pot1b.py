#!/usr/bin/env python3
"""Carry a knob-pinched, hanging-vertical pot to (tx,ty) on the stove and set it down.
Usage: python3 -u pot1b.py <tx> <ty> [knob_above_base=0.150]
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from geom import Rz
from rob import Robot, log

tx, ty = map(float, sys.argv[1:3])
KAB = float(sys.argv[3]) if len(sys.argv) > 3 else 0.150
HZ = 0.093
PLATE_Z = float(sys.argv[4]) if len(sys.argv) > 4 else 0.9304
CARRY_Z = 1.16
BASE = np.array([-0.66, 0.0])
r = Robot("pot1b")


def azim(K):
    return np.arctan2(K[1] - BASE[1], K[0] - BASE[0])


def plan_carry(q0, K0, K1, R0, step=0.03):
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


def chunks(path, n, secs, what, fatal=True):
    for i in range(0, len(path), n):
        ok = r.move_path(path[i:i + n], secs)
        f = r.fingers()
        log(f"{what} chunk ok={ok} gap {f[0]-f[1]:.4f}")
        if not ok and not fatal:
            log(f"{what}: stalled, continuing"); return
        if not ok or f[0] - f[1] < 0.004:
            log(f"ABORT during {what}"); sys.exit(1)


p, qu = r.fk()
R0 = Rot.from_quat(qu).as_matrix()
assert abs(R0[2, 2]) < 0.1, "hand is not horizontal"
K0 = p + HZ * R0[:, 2]
K_up = np.array([K0[0], K0[1], CARRY_Z])
K_carry = np.array([tx, ty, CARRY_Z])
K_down = np.array([tx, ty, PLATE_Z + KAB + 0.002])
q = r.arm_q()
up = r.plan_line(q, K_up - HZ * R0[:, 2], R0, step=0.03); assert up is not None, "up plan"
carry, R_T = plan_carry(up[-1], K_up, K_carry, R0); assert carry is not None, "carry plan"
down = r.plan_line(carry[-1], K_down - HZ * R_T[:, 2], R_T); assert down is not None, "down plan"
p_ret = K_down - HZ * R_T[:, 2] - 0.08 * R_T[:, 2] + [0, 0, 0.06]
ret = r.plan_line(down[-1], p_ret, R_T); assert ret is not None, "retreat plan"
log(f"plans ok: up {len(up)} carry {len(carry)} down {len(down)} ret {len(ret)}; hand z at stove {np.round(R_T[:,2],3)}")

chunks(up, 6, 4.0, "up")
chunks(carry, 6, 4.0, "carry")
chunks(down, 6, 4.0, "down", fatal=False)
p, qu = r.fk(); Rn = Rot.from_quat(qu).as_matrix()
log(f"before release: pinch at {np.round(p + HZ * Rn[:,2],4)} fingers {r.fingers()}")
r.gripper(0.04)
ret = r.plan_line(r.arm_q(), p_ret, R_T); assert ret is not None, "retreat replan"
ok = r.move_path(ret, 4.0)
log(f"retreat ok={ok}")
log("PLACE DONE")
