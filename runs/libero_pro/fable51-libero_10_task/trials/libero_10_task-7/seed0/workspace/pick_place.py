#!/usr/bin/env python3
"""Pick one object top-down and drop it into the basket.

Usage: python3 -u pick_place.py <x> <y> <grasp_tcp_z> <yaw_deg> <bx> <by> <release_tcp_z>
yaw_deg: world yaw of the finger-opening axis (0 = fingers along world X).
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from arm import Arm, GRIP, TCP_OFF

HOVER_Z = 0.62         # pre-grasp TCP height
TRANSIT_Z = 0.80       # carry height (hanging object must clear the rim)


def finger_axis_yaw(a):
    p, quat = a.fk_world()
    R = Rot.from_quat(quat).as_matrix()
    y = R[:, 1]  # hand Y = finger axis
    return np.degrees(np.arctan2(y[1], y[0])), p + TCP_OFF * R[:, 2]


def main():
    x, y, gz, yaw, bx, by, rz = map(float, sys.argv[1:8])
    a = Arm()
    print("start joints", np.round(a.arm_q(), 3), flush=True)

    print("open gripper", flush=True)
    a.gripper(GRIP["open_m"])

    print(f"pre-grasp above ({x},{y}) at z={HOVER_Z}, yaw {yaw}", flush=True)
    q_hover = a.ik_best([x, y, HOVER_Z], yaw)
    a.move_q(q_hover, seconds=4.0)
    fy, tcp = finger_axis_yaw(a)
    print(f"  TCP {tcp.round(4)} finger-axis yaw {fy:.1f} (want {yaw} mod 180)", flush=True)
    if abs(((fy - yaw) + 90) % 180 - 90) > 5:
        print("YAW OFF: aborting before descent", flush=True)
        sys.exit(3)

    print(f"descend to grasp z={gz}", flush=True)
    mid = (HOVER_Z + gz) / 2
    q_mid = a.ik_best([x, y, mid], yaw, seed=q_hover, n=2)
    q_grasp = a.ik_best([x, y, gz], yaw, seed=q_mid, n=2)
    a.move_q(q_grasp, seconds=4.0, via=[(q_mid, 2.0)])
    fy, tcp = finger_axis_yaw(a)
    print(f"  TCP {tcp.round(4)} finger-axis yaw {fy:.1f} wrench {a.wrench()}", flush=True)

    print("close gripper", flush=True)
    g = a.gripper(GRIP["closed_m"])
    gap = abs(g[0]) + abs(g[1])
    print(f"  finger gap = {gap:.4f} m", flush=True)
    if gap < 0.005:
        print("GRASP FAILED: fingers closed on air", flush=True)
        a.gripper(GRIP["open_m"])
        a.move_q(q_hover, seconds=3.0)
        sys.exit(2)

    print("lift", flush=True)
    a.move_q(q_hover, seconds=3.0)
    g = a.finger_gap()
    print("  fingers after lift", np.round(g, 4), flush=True)
    if abs(g[0]) + abs(g[1]) < 0.005:
        print("LOST OBJECT on lift", flush=True)
        sys.exit(2)

    print("transit to basket", flush=True)
    q_up = a.ik_best([x, y, TRANSIT_Z], yaw, seed=q_hover, n=2)
    q_b_hi = a.ik_best([bx, by, TRANSIT_Z], yaw, seed=q_up, n=2)
    q_b = a.ik_best([bx, by, rz], yaw, seed=q_b_hi, n=2)
    a.move_q(q_up, seconds=2.0)
    a.move_q(q_b_hi, seconds=4.0)
    print("  fingers over basket", np.round(a.finger_gap(), 4), flush=True)
    a.move_q(q_b, seconds=3.0)
    fy, tcp = finger_axis_yaw(a)
    print(f"  TCP {tcp.round(4)} fingers {np.round(a.finger_gap(), 4)}", flush=True)

    print("release", flush=True)
    a.gripper(GRIP["open_m"])

    print("retreat up", flush=True)
    a.move_q(q_b_hi, seconds=3.0)
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
