#!/usr/bin/env python3
"""topbar.py: vertical pinch of the mug's top handle bar, lift, rotate/translate, set down.
usage: topbar.py cx cy az_deg  ncx ncy naz_deg [mode]   (mode: check|grasp|run)
cx,cy: mug axis; az: handle azimuth (deg, from +x). new center/azimuth for set-down."""
import sys, numpy as np
from rob import Robot, quat_from_axes

cx, cy, az, ncx, ncy, naz = map(float, sys.argv[1:7])
mode = sys.argv[7] if len(sys.argv) > 7 else "check"
R_BAR = 0.062          # top bar centre radius
Z_GRASP = 0.968
OFF = 10.0             # closing axis rotated this many deg from perpendicular (hand body away from the face)

def hand_quat(az_deg):
    u = np.array([np.cos(np.radians(az_deg)), np.sin(np.radians(az_deg)), 0])      # bar direction (outward)
    xh = np.array([np.cos(np.radians(az_deg - OFF)), np.sin(np.radians(az_deg - OFF)), 0])
    return quat_from_axes([0, 0, -1], xh), u

r = Robot("topbar")
q_now = r.q()
quat, u = hand_quat(az)
tcp = np.array([cx, cy, 0]) + R_BAR * u
nquat, nu = hand_quat(naz)
ntcp = np.array([ncx, ncy, 0]) + R_BAR * nu
print("grasp tcp", np.round(tcp, 4), "new tcp", np.round(ntcp, 4))

def go(p, qu, sec=3.0, tol=0.003, iters=4):
    tgt = np.array(p, float); q = None
    for it in range(iters):
        cur, _ = r.tcp_pose()
        if it == 0:
            cmd = tgt.copy()
        else:
            off = cur - tgt
            if np.linalg.norm(off) < tol:
                break
            cmd = cmd - off
        q = r.solve_ik(cmd, qu, seed=r.q(), tries=10)
        if q is None:
            print("  IK FAIL at", np.round(cmd, 4)); return False
        code, err = r.move_joints(q, seconds=sec if it == 0 else 2.0)
        cur, _ = r.tcp_pose()
        print(f"  it{it} code {code} err {err:.4f} tcp {np.round(cur,4)} off {np.round(cur-tgt,4)}")
    return True

poses = [("pre", [tcp[0], tcp[1], 1.10], quat), ("grasp", [tcp[0], tcp[1], Z_GRASP], quat),
         ("lift", [tcp[0], tcp[1], Z_GRASP + 0.04], quat), ("rot", [ntcp[0], ntcp[1], Z_GRASP + 0.04], nquat),
         ("down", [ntcp[0], ntcp[1], Z_GRASP + 0.002], nquat), ("up", [ntcp[0], ntcp[1], 1.10], nquat)]
if mode == "check":
    seed = q_now
    for name, p, qu in poses:
        q = r.solve_ik(p, qu, seed=seed, tries=10)
        print(name, np.round(p, 3), "OK" if q is not None else "IK FAIL", np.round(q, 2) if q is not None else "")
        if q is not None: seed = q
    sys.exit()

print("open", r.gripper(0.08))
for name, p, qu in poses:
    print(name, np.round(p, 4))
    ok = go(p, qu, sec=4.0 if name in ("pre", "up") else 3.0)
    if not ok: sys.exit(1)
    if name == "grasp":
        if mode == "grasp":
            sys.exit()
        print("close", r.gripper(0.0))
        f = r.fingers()
        if abs(f[0]) < 0.006:
            print("GRASP MISSED (fingers %.4f), opening" % f[0]); r.gripper(0.08); sys.exit(1)
    if name == "lift":
        print("fingers after lift", r.fingers())
    if name == "down":
        print("open", r.gripper(0.08))
print("done", r.fingers())
