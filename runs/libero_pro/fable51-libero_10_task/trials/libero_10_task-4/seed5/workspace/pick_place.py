#!/usr/bin/env python3
"""Rim-pinch pick of a mug and place onto a plate.

Usage: python3 pick_place.py <mug_cx> <mug_cy> <rim_z> <wall_r> <side:+y|-y|+x|-x> <plate_x> <plate_y> [tag]
The pinch point is the mug wall on the given side of the rim centre; one
finger goes inside the mug, one outside.
"""
import sys

import numpy as np

from robot import Robot, down_quat

TABLE_Z = 0.423
PLATE_TOP = 0.455
CARRY_Z = 0.78
GRASP_DEPTH = 0.035  # fingertips this far below the rim


def main():
    cx, cy, rim_z, wall_r = map(float, sys.argv[1:5])
    side = sys.argv[5]
    px, py = float(sys.argv[6]), float(sys.argv[7])
    r = Robot("pick_place")
    off = {"+y": (0, wall_r), "-y": (0, -wall_r), "+x": (wall_r, 0), "-x": (-wall_r, 0)}[side]
    yaw = 0.0 if side in ("+y", "-y") else 90.0
    qd = down_quat(yaw)
    gx, gy = cx + off[0], cy + off[1]
    grasp_z = rim_z - GRASP_DEPTH
    mug_bottom_below_tcp = grasp_z - TABLE_Z

    print("== open gripper")
    r.open()
    print("== hover above pinch point")
    r.move_tcp([gx, gy, rim_z + 0.12], qd, 3.0)
    print("== descend")
    r.move_tcp([gx, gy, grasp_z], qd, 2.5)
    p, _ = r.tcp()
    if np.linalg.norm(p[:2] - [gx, gy]) > 0.006 or abs(p[2] - grasp_z) > 0.006:
        print("!! descend inaccurate", p)
    f0, _ = r.wrench()
    print("== close")
    gap = r.close()
    f1, _ = r.wrench()
    print(f"  gap after close {gap:.4f} (closed_m=0 -> >0 means holding); wrench dF={np.round(f1 - f0, 2)}")
    if gap < 0.003:
        print("!! nothing grasped")
        sys.exit(2)
    print("== lift")
    r.move_tcp([gx, gy, CARRY_Z], qd, 3.0)
    gap2 = r.finger_gap()
    print(f"  gap after lift {gap2:.4f}")
    if gap2 < 0.003:
        print("!! lost the mug on lift")
        sys.exit(3)
    print("== carry above plate")
    tx, ty = px + off[0], py + off[1]
    r.move_tcp([tx, ty, CARRY_Z], qd, 4.0)
    print("== lower onto plate")
    rel_z = PLATE_TOP + mug_bottom_below_tcp + 0.008
    r.move_tcp([tx, ty, rel_z + 0.05], qd, 2.5)
    r.move_tcp([tx, ty, rel_z], qd, 2.0)
    print("== release")
    r.open()
    print("== retreat")
    r.move_tcp([tx, ty, rel_z + 0.15], qd, 2.5)
    print("done")


if __name__ == "__main__":
    main()
