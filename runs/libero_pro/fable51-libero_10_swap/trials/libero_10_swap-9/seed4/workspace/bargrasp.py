#!/usr/bin/env python3
"""Vertical pinch of a horizontal bar lying along direction (bx,by): closing axis n = perp."""
import sys, numpy as np
from rob import Robot, quat_from_axes
r = Robot("bar")
cx, cy, bx, by, z, off = map(float, sys.argv[1:7])   # bar center, bar dir, tcp z, tcp offset along n (neg = toward -n)
mode = sys.argv[7] if len(sys.argv) > 7 else "run"
b = np.array([bx, by, 0]); b /= np.linalg.norm(b)
n = np.array([-b[1], b[0], 0])
if n[1] < 0: n = -n            # n points +y-ish
quat = quat_from_axes([0, 0, -1], b)   # hand x = bar dir, hand y (closing) = +-n
tcp = np.array([cx, cy, 0]) + off * n
def go(p, sec=4.0, tries=3):
    p = np.array(p, float); cmd = p.copy()
    for it in range(5):
        q = r.solve_ik(cmd, quat, seed=r.q(), tries=10)
        if q is None: print("IK FAIL", cmd); sys.exit(1)
        for a in range(tries):
            code, err = r.move_joints(q, sec if it == 0 else 3.0)
            if err < 0.03: break
        pn, _ = r.tcp_pose(); off = pn - p
        print(f"  it{it} at {np.round(pn,4).tolist()} code {code} err {round(err,4)} off {np.round(off,4).tolist()}")
        if np.linalg.norm(off) < 0.003: break
        cmd = cmd - off
    print("at", np.round(pn, 4).tolist(), "fingers", np.round(r.fingers(), 4).tolist())
    return pn
print("n", np.round(n, 3), "tcp xy", np.round(tcp[:2], 4))
if mode == "check":
    for zz in [1.03, z]:
        q = r.solve_ik([tcp[0], tcp[1], zz], quat, seed=r.q(), tries=10); print(zz, q is not None)
    sys.exit()
go([tcp[0], tcp[1], 1.03])
p = go([tcp[0], tcp[1], z], 5.0)
if abs(p[2] - z) > 0.006 or np.linalg.norm(p[:2] - tcp[:2]) > 0.004:
    p = go([tcp[0], tcp[1], z] - (p - [tcp[0], tcp[1], z]), 3.0)
if mode == "down": sys.exit()
print("close", r.gripper(0.0))
go([tcp[0], tcp[1], z + 0.04], 3.0)
print("fingers after lift", np.round(r.fingers(), 4).tolist())
