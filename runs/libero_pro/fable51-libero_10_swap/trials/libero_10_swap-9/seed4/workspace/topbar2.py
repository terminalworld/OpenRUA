#!/usr/bin/env python3
"""topbar2.py cx cy az ncx ncy naz [check|run] -- like topbar.py but with configuration-consistent IK."""
import sys, numpy as np
from rob import Robot, quat_from_axes
from ikbest import ik_best

cx, cy, az, ncx, ncy, naz = map(float, sys.argv[1:7])
mode = sys.argv[7] if len(sys.argv) > 7 else "check"
R_BAR = 0.062; Z_GRASP = 0.968; OFF = 10.0

FLIP1 = float(sys.argv[8]) if len(sys.argv) > 8 else 0.0
FLIP2 = float(sys.argv[9]) if len(sys.argv) > 9 else 0.0
def hand_quat(az_deg, flip=0.0):
    u = np.array([np.cos(np.radians(az_deg)), np.sin(np.radians(az_deg)), 0])
    a = np.radians(az_deg - OFF + flip)
    xh = np.array([np.cos(a), np.sin(a), 0])
    return quat_from_axes([0, 0, -1], xh), u

r = Robot("topbar2")
quat, u = hand_quat(az, FLIP1); tcp = np.array([cx, cy, 0]) + R_BAR * u
nquat, nu = hand_quat(naz, FLIP2); ntcp = np.array([ncx, ncy, 0]) + R_BAR * nu
poses = [("pre", [tcp[0], tcp[1], 1.10], quat), ("grasp", [tcp[0], tcp[1], Z_GRASP], quat),
         ("lift", [tcp[0], tcp[1], Z_GRASP + 0.04], quat), ("rot", [ntcp[0], ntcp[1], Z_GRASP + 0.04], nquat),
         ("down", [ntcp[0], ntcp[1], Z_GRASP + 0.002], nquat), ("up", [ntcp[0], ntcp[1], 1.10], nquat)]
ref = r.q()
plan = []
for name, p, qu in poses:
    q, d = ik_best(r, p, qu, ref)
    print(name, np.round(p, 3), "IK FAIL" if q is None else f"d={d:.2f} q={np.round(q,2)}")
    if q is None: sys.exit(1)
    plan.append((name, p, qu, q)); ref = q
if mode == "check": sys.exit()

def go(p, qu, qseed, sec):
    tgt = np.array(p, float); cmd = tgt.copy(); q = qseed
    for it in range(4):
        if it > 0:
            cur, _ = r.tcp_pose(); off = cur - tgt
            if np.linalg.norm(off) < 0.003: break
            cmd = cmd - off
            q, _ = ik_best(r, cmd, qu, r.q(), n=4)
            if q is None: print("  IK FAIL corr"); return False
        code, err = r.move_joints(q, seconds=sec if it == 0 else 2.0)
        cur, _ = r.tcp_pose()
        print(f"  it{it} code {code} err {err:.4f} tcp {np.round(cur,4)} off {np.round(cur-tgt,4)} fingers {np.round(r.fingers(),4)}"); sys.stdout.flush()
    return True

print("open", r.gripper(0.08))
for name, p, qu, q in plan:
    print(name, np.round(p, 4))
    dq = np.abs(q - r.q()).max(); sec = max(2.0, min(8.0, dq * 3.0))
    go(p, qu, q, sec)
    if name == "grasp":
        print("close", r.gripper(0.0)); f = r.fingers()
        if min(abs(f[0]), abs(f[1])) < 0.006:
            print("BAD GRASP"); print(r.gripper(0.08)); sys.exit(1)
    if name == "down":
        print("open", r.gripper(0.08))
print("done", r.fingers())
