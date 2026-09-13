#!/usr/bin/env python3
"""Grasp the lying alphabet-soup can with a rolled hand (fingers close
across the can axis in the y-z plane, wrist displaced toward -y), then
drop it into the basket."""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rob import Robot, Q_DOWN
from pick_place import go, cart_path, log, BASKET

X, Y, ZC = -0.18, -0.154, 0.454          # can centre (world)
ROLL = np.radians(25)
GRASP_Z = 0.458
PRE_Z = 0.60
LIFT_Z = 0.76


def q_roll(alpha):
    R = Rot.from_euler('x', alpha).as_matrix() @ np.diag([1, -1, -1])
    return tuple(Rot.from_matrix(R).as_quat())


def main():
    r = Robot("pick_lying")
    QG = q_roll(ROLL)
    log("start", r.tcp_world().round(3), r.fingers())
    log("open", r.gripper(0.04))

    pre = cart_path(r, r.joints(), [(X, Y, LIFT_Z), (X, Y, PRE_Z)], QG)
    log("-> pre"); go(r, pre, 6.0)

    zs = np.linspace(PRE_Z, GRASP_Z, 5)[1:]
    down = cart_path(r, pre[-1], [(X, Y, z) for z in zs], QG)
    log("-> descend"); tcp = go(r, down, 3.0)
    err = np.linalg.norm(tcp - np.array([X, Y, GRASP_Z]))
    log(f"  tcp err={err:.4f}")
    if err > 0.012:
        log("  ABORT: descent blocked, lifting")
        go(r, cart_path(r, r.joints(), [(tcp[0], tcp[1], PRE_Z)], QG), 3.0)
        sys.exit(3)

    log("close"); res = r.gripper(0.0); log(" ", res)
    gap = abs(res[2][0]) + abs(res[2][1])
    log(f"  gap={gap:.4f}")
    if gap < 0.01:
        log("  GRASP FAILED"); sys.exit(2)

    up = cart_path(r, r.joints(), [(X, Y, GRASP_Z + 0.08), (X, Y, LIFT_Z)], QG)
    log("-> lift"); go(r, up, 3.0)
    log("  fingers", r.fingers())
    if abs(r.fingers()[0]) + abs(r.fingers()[1]) < 0.01:
        log("  DROPPED"); sys.exit(2)

    over = cart_path(r, r.joints(), [(BASKET[0], BASKET[1], LIFT_Z)], Q_DOWN)
    log("-> over basket"); go(r, over, 6.0)
    log("  fingers", r.fingers())
    lower = cart_path(r, r.joints(), [(BASKET[0], BASKET[1], 0.70)], Q_DOWN)
    log("-> lower"); go(r, lower, 2.0)
    log("open", r.gripper(0.04))
    back = cart_path(r, r.joints(), [(BASKET[0], BASKET[1], LIFT_Z + 0.04)], Q_DOWN)
    log("-> retreat"); go(r, back, 2.0)
    log("DONE", r.fingers())


if __name__ == "__main__":
    main()
