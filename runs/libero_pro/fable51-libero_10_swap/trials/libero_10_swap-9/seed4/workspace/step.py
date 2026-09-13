#!/usr/bin/env python3
"""step.py <cmd> [args]  -- one-shot robot actions built on rob.py
  tcp x y z th [sx] [sec]  : move TCP to world pos with hand tilted th deg toward -y (hand y = sx*world x)
  down                      : hand pointing straight down: tcp x y z (yaw 0 -> hand x = world x, closing along y)
  grip open|close
  state                     : print q, tcp pose, fingers
  snap cam [out]
"""
import sys, json
import numpy as np
from rob import Robot, quat_from_axes, quat_down, TCP
from scipy.spatial.transform import Rotation as Rot


def tilt_quat(th, sx=1):
    t = np.radians(th)
    z = [0, np.sin(t), -np.cos(t)]
    x = [0, sx * np.cos(t), sx * np.sin(t)]
    return quat_from_axes(z, x)


def dir_quat(th, yaw, sx=1):
    """hand pointing along horizontal dir d (yaw deg about z from +y, negative -> toward -x),
    pitched th deg from vertical; hand y (closing) = horizontal perpendicular to d."""
    t = np.radians(th); yw = np.radians(yaw)
    d = np.array([-np.sin(yw), np.cos(yw), 0])
    z = d*np.sin(t) + np.array([0, 0, -np.cos(t)])
    x = sx*(d*np.cos(t) + np.array([0, 0, np.sin(t)]))
    return quat_from_axes(z, x)


def main():
    a = sys.argv[1:]
    r = Robot("step")
    cmd = a[0]
    if cmd == "state":
        q = r.q(); p, qu = r.tcp_pose(q)
        print("q", np.round(q, 4).tolist()); print("tcp", np.round(p, 4).tolist(), "quat", np.round(qu, 3).tolist())
        print("fingers", r.fingers())
    elif cmd == "grip":
        w = 0.08 if a[1] == "open" else 0.0
        print("grip", r.gripper(w))
    elif cmd == "snap":
        print(r.snap(a[1], a[2] if len(a) > 2 else None))
    elif cmd in ("tcp", "vert", "pose"):
        x, y, z = map(float, a[1:4])
        if cmd == "pose":
            th = float(a[4]); yaw = float(a[5]); sx = int(a[6]) if len(a) > 6 else 1
            sec = float(a[7]) if len(a) > 7 else 4.0
            quat = dir_quat(th, yaw, sx)
        elif cmd == "tcp":
            th = float(a[4]); sx = int(a[5]) if len(a) > 5 else 1
            sec = float(a[6]) if len(a) > 6 else 4.0
            quat = tilt_quat(th, sx)
        else:
            # vertical: hand z = -world z, hand y = world x  (closing along x)
            ax = a[4] if len(a) > 4 else "x"
            xdir = {"x": [0, 1, 0], "xm": [0, -1, 0], "y": [1, 0, 0], "ym": [-1, 0, 0]}[ax]  # x/xm: close along world x
            quat = quat_from_axes([0, 0, -1], xdir)
            sec = float(a[5]) if len(a) > 5 else 4.0
        seed = r.q()
        q = r.solve_ik([x, y, z], quat, seed=seed, tries=10)
        if q is None:
            print("IK FAIL"); return
        dq = np.abs(q - seed).max()
        print("q_target", np.round(q, 3).tolist(), "max dq", round(dq, 3))
        for attempt in range(3):
            code, err = r.move_joints(q, seconds=sec)
            print("move code", code, "err", round(err, 4))
            if err < 0.01:
                break
        p, qu = r.tcp_pose()
        print("tcp now", np.round(p, 4).tolist(), "quat", np.round(qu, 3).tolist())
        # closed-loop correction of steady-state offset
        tgt = np.array([x, y, z])
        for it in range(3):
            off = p - tgt
            if np.linalg.norm(off) < 0.003:
                break
            q = r.solve_ik(tgt - off, quat, seed=r.q(), tries=10)
            if q is None:
                print("IK FAIL (corr)"); break
            code, err = r.move_joints(q, seconds=2.0)
            p, qu = r.tcp_pose()
            print(f"corr{it} off={np.round(off,4).tolist()} -> tcp {np.round(p,4).tolist()} code {code} err {round(err,4)}")
        print("fingers", r.fingers())
    elif cmd == "joints":
        q = np.array(json.loads(a[1])); sec = float(a[2]) if len(a) > 2 else 4.0
        for attempt in range(3):
            code, err = r.move_joints(q, seconds=sec)
            print("move code", code, "err", round(err, 4))
            if err < 0.01:
                break
        p, qu = r.tcp_pose(); print("tcp now", np.round(p, 4).tolist())


if __name__ == "__main__":
    main()
