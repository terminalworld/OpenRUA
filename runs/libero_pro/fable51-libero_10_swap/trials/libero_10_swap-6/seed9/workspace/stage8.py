#!/usr/bin/env python3
"""Right the knocked-over red mug.

Lying-cylinder geometry (from point clouds): axis dir a, centre c, axis
height ZC, outer radius R0, mouth plane at +MOUTH along a from c.
Plan: pitched head-on rim pinch at the top of the mouth (one finger inside,
one outside), lift, rotate 90 deg about n so the axis points up, place.

Usage: python3 stage8.py plan|A|B|C
"""
import math
import sys

import numpy as np

from arm import Arm, HOME, R_quat, rot_axis

a = np.array([0.0, -1.0, 0.0]); a /= np.linalg.norm(a)   # attempt 4: mug lies along y, mouth faces -y
n = np.array([-a[1], a[0], 0.0])
Z = np.array([0.0, 0.0, 1.0])
c = np.array([-0.022, 0.034, 0.0])
ZC, R0, WALL = 0.4755, 0.05, 0.005
MOUTH = 0.074
BETA = math.radians(0.0)   # attempt 3: horizontal hand, vertical fingers
TABLE = 0.4255
MUG_H = 0.148

# grasp point: 2 cm inside mouth, slightly to +n, wall mid-thickness
PERP = 0.0
zg = 0.541   # hand shell (~0.113 below TCP axis) just clears the table; lower finger must enter below the inner rim top (0.521)
G = c + (MOUTH - 0.015) * a + PERP * n + np.array([0, 0, zg])

d1 = -math.cos(BETA) * a - math.sin(BETA) * Z          # approach: into mouth, down
f1 = -math.sin(BETA) * a + math.cos(BETA) * Z          # finger axis: up/back


def R_from(d, f):
    x = np.cross(f, d)
    return np.column_stack([x, f, d])


def rotated(R, ang):
    return rot_axis(n, ang) @ R


R1 = R_from(d1, f1)
R1b = R_from(d1, -f1)
PRE = G - 0.07 * d1   # straight back along the approach axis
LIFT = G + np.array([0, 0, 0.20])
TARGET_AXIS = np.array([-0.02, 0.16])
STEPS = 6


def place_tcp(R):
    # after rotation, grasp point is at axis - R0*(rotated +Z)
    rad = R @ np.linalg.inv(R1) @ Z  # world dir of original +Z after rotation
    xy = TARGET_AXIS + (R0 - WALL / 2) * rad[:2]
    return np.array([xy[0], xy[1], TABLE + MUG_H - 0.02 + 0.006])


def main():
    mode = sys.argv[1]
    arm = Arm("stage8")
    print("G", G.round(4), "PRE", PRE.round(4), flush=True)
    if mode == "plan":
        cur = np.array(arm.arm_q())
        for name, R in [("R1", R1), ("R1b", R1b)]:
            q = arm.ik_best(PRE, R_quat(R))
            print(name, "pre-grasp IK", None if q is None else np.round(q, 3),
                  None if q is None else round(float(np.abs(np.array(q) - cur).max()), 3))
            if q is None:
                continue
            seed = q
            chain = [(G, R_quat(R)), (LIFT, R_quat(R))]
            for k in range(1, STEPS + 1):
                chain.append((LIFT, R_quat(rotated(R, -k * math.pi / 2 / STEPS))))
            Rf = rotated(R, -math.pi / 2)
            P = place_tcp(Rf)
            chain.append((np.array([P[0], P[1], LIFT[2]]), R_quat(Rf)))
            chain.append((P, R_quat(Rf)))
            ok = True
            for xyz, quat in chain:
                s = arm.ik_world(xyz, quat, seed=seed, tries=2)
                if s is None:
                    print("   IK fail at", np.round(xyz, 3)); ok = False; break
                print("   step dq=%.3f" % np.abs(np.array(s) - np.array(seed)).max(), np.round(s, 2))
                seed = s
            print(name, "chain ok" if ok else "chain FAILED", "place", P.round(4))
        return

    R = R1b if (len(sys.argv) > 2 and sys.argv[2] == "b") else R1
    if mode == "A":
        arm.open()
        p0, q0 = arm.tcp_world()
        arm.move_path([(p0 + np.array([0, 0, 0.26]), q0)], secs=5)   # straight up first
        HI = PRE + np.array([0, 0, 0.27])
        qPRE = arm.ik_best(PRE, R_quat(R), n=8)   # pick the branch at PRE, then HI in that branch
        qHI = arm.ik_world(HI, R_quat(R), seed=qPRE)
        if qHI is None:
            print("IK FAILED for HI", flush=True); return
        arm.move_joints(qHI, secs=10)
        arm.move_path([(HI + (PRE - HI) * i / 4, R_quat(R)) for i in range(1, 5)], secs=7)
        p, q = arm.tcp_world(); print("at pre: tcp", p.round(4), "quat", np.round(q, 3), flush=True)
        # straight approach along d1 to the grasp point
        arm.move_path([(PRE + (G - PRE) * i / 4, R_quat(R)) for i in range(1, 5)], secs=6)
        p, q = arm.tcp_world(); print("at grasp: tcp", p.round(4), "quat", np.round(q, 3), flush=True)
        arm.close()
        print("fingers", arm.finger_gap(), flush=True)
    elif mode == "B":
        arm.move_path([(G + (LIFT - G) * i / 4, R_quat(R)) for i in range(1, 5)], secs=6)
        poses = [(LIFT, R_quat(rotated(R, -k * math.pi / 2 / STEPS))) for k in range(1, STEPS + 1)]
        arm.move_path(poses, secs=10)
        print("fingers", arm.finger_gap(), flush=True)
    elif mode == "C":
        Rf = rotated(R, -math.pi / 2)
        # measured while hanging (argv): axis = TCP + (ox, oy); bottom = TCP z - dz
        ox, oy, dz = map(float, sys.argv[3:6])
        P = np.array([TARGET_AXIS[0] - ox, TARGET_AXIS[1] - oy, TABLE + dz + 0.001])
        p0, _ = arm.tcp_world()
        P1 = np.array([P[0], P[1], p0[2]])
        arm.move_path([(p0 + (P1 - p0) * i / 4, R_quat(Rf)) for i in range(1, 5)], secs=6)
        arm.move_path([(P1 + (P - P1) * i / 6, R_quat(Rf)) for i in range(1, 7)], secs=10)
        p, q = arm.tcp_world(); print("at place: tcp", p.round(4), "quat", np.round(q, 3), flush=True)
        arm.open()
        arm.spin(1.0)
        print("fingers", arm.finger_gap(), flush=True)
        p2, _ = arm.tcp_world()
        up = p2 + np.array([0, 0, 0.15])
        arm.move_path([(p2 + (up - p2) * i / 6, R_quat(Rf)) for i in range(1, 7)], secs=14)
    elif mode == "home":
        arm.move_joints(HOME, secs=8)
    print("done", flush=True)


if __name__ == "__main__":
    main()
