#!/usr/bin/env python3
"""outerbar.py: tilted (th deg, fingers pointing +y-down) pinch of the handle's outer bar.
usage: outerbar.py grasp gx gy gz        -> open, above, grasp, close, lift 4cm
       outerbar.py carry                  -> lift, carry, down, in, rel(open), retreat
       outerbar.py go x y z [sec]         -> single move with current quat"""
import sys, numpy as np
from rob import Robot
from step import tilt_quat

TH = 30
quat = tilt_quat(TH, -1)
r = Robot("outerbar")

def go(p, sec=3.0, tol=0.003, iters=4):
    tgt = np.array(p, float); cmd = tgt.copy()
    for it in range(iters):
        if it > 0:
            cur, _ = r.tcp_pose(); off = cur - tgt
            if np.linalg.norm(off) < tol: break
            cmd = cmd - off
        q = r.solve_ik(cmd, quat, seed=r.q(), tries=10)
        if q is None:
            print("  IK FAIL at", np.round(cmd, 4)); return False
        code, err = r.move_joints(q, seconds=sec if it == 0 else 2.0)
        cur, _ = r.tcp_pose()
        print(f"  it{it} code {code} err {err:.4f} tcp {np.round(cur,4)} off {np.round(cur-tgt,4)} fingers {np.round(r.fingers(),4)}")
        sys.stdout.flush()
    return True

def line(p0, p1, step=0.02, sec_per=1.0):
    """straight-line cartesian move in small steps"""
    p0 = np.array(p0, float); p1 = np.array(p1, float)
    n = max(1, int(np.ceil(np.linalg.norm(p1 - p0) / step)))
    for i in range(1, n + 1):
        p = p0 + (p1 - p0) * i / n
        q = r.solve_ik(p, quat, seed=r.q(), tries=10)
        if q is None:
            print("  IK FAIL at", np.round(p, 4)); return False
        code, err = r.move_joints(q, seconds=sec_per)
        if code != 0 and err > 0.02:
            cur, _ = r.tcp_pose(); print(f"  step {i}/{n} code {code} err {err:.4f} tcp {np.round(cur,4)}"); return False
    cur, _ = r.tcp_pose(); print(f"  line end tcp {np.round(cur,4)} fingers {np.round(r.fingers(),4)}")
    return True

mode = sys.argv[1]
if mode == "grasp":
    gx, gy, gz = map(float, sys.argv[2:5])
    print("open", r.gripper(0.08))
    print("above"); go([gx, gy, 1.02], sec=4.0)
    print("grasp"); go([gx, gy, gz], sec=3.0)
    print("close", r.gripper(0.0))
    f = r.fingers()
    if min(abs(f[0]), abs(f[1])) < 0.006:
        print("BAD GRASP, opening"); print(r.gripper(0.08)); sys.exit(1)
    print("lift"); go([gx, gy, gz + 0.04], sec=2.0)
    print("fingers", r.fingers())
elif mode == "go":
    x, y, z = map(float, sys.argv[2:5]); sec = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
    go([x, y, z], sec=sec)
elif mode == "line":
    x, y, z = map(float, sys.argv[2:5])
    cur, _ = r.tcp_pose(); line(cur, [x, y, z])
elif mode == "carry":
    print("lift"); go([-0.203, -0.480, 1.20], sec=4.0)
    print("carry"); go([-0.162, -0.475, 1.20], sec=3.0)
    print("down"); go([-0.162, -0.475, 1.005], sec=4.0)
    print("in"); ok = line([-0.162, -0.475, 1.005], [-0.162, -0.335, 1.005], step=0.02, sec_per=1.0)
    if not ok: sys.exit(1)
    go([-0.162, -0.335, 1.005], sec=1.0)
    print("fingers", r.fingers())
