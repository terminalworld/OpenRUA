#!/usr/bin/env python3
"""Rim-grasp a mug and place it on a plate, all moves via MoveIt Cartesian
paths (joint-continuous). Args as pick_place.py.

Usage: python3 pick_place_cart.py <mug_x> <mug_y> <rim_z> <side> <plate_x> <plate_y>
"""
import sys
import numpy as np
from ctl import Ctl, DOWN

WALL_R = 0.041
GRASP_BELOW_RIM = 0.03
TABLE_Z = 0.422
PLATE_TOP = 0.456
CARRY_Z = 0.70
OPEN, CLOSED = 0.04, 0.0


def main():
    mx, my, rim_z, side, px, py = map(float, sys.argv[1:7])
    mug_h = rim_z - TABLE_Z
    grasp = np.array([mx, my + side * WALL_R, rim_z - GRASP_BELOW_RIM])
    pre = grasp.copy(); pre[2] = CARRY_Z
    place = np.array([px, py + side * WALL_R, PLATE_TOP + (mug_h - GRASP_BELOW_RIM) + 0.008])
    place_pre = place.copy(); place_pre[2] = CARRY_Z
    print("grasp", grasp, "place", place)

    c = Ctl()

    def go(target, speed, tol=0.008):
        c.move_cart(target, DOWN, speed=speed)
        pos, _ = c.tcp_world()
        err = np.linalg.norm(pos - target)
        if err > tol:
            print(f"  off by {err:.4f}, correcting"); c.move_cart(target, DOWN, speed=0.03)
            pos, _ = c.tcp_world(); err = np.linalg.norm(pos - target)
        return err

    print("[0] open gripper"); c.gripper(OPEN)
    print("[1] to pre-grasp", pre); go(pre, 0.1)
    w0 = c.wrench(); print("  wrench", w0)
    print("[2] descend to just above rim");
    above = grasp.copy(); above[2] = rim_z + 0.03
    err = go(above, 0.06)
    if err > 0.01:
        print("ABORT: cannot reach above-rim pose accurately"); c.close(); sys.exit(4)
    print("[2b] descend to grasp", grasp); err = go(grasp, 0.04)
    w1 = c.wrench(); print("  wrench", w1, "delta", None if w0 is None else np.round(w1 - w0, 2))
    print("[3] close"); f = c.gripper(CLOSED)
    gap = f[0] - f[1]
    print(f"  finger gap {gap:.4f} (0 => closed on air)")
    if gap < 0.002:
        print("GRASP FAILED: nothing held"); c.gripper(OPEN); go(pre, 0.06); c.close(); sys.exit(2)
    print("[4] lift", pre); go(pre, 0.06)
    f = c.fingers(); print(f"  gap after lift {f[0]-f[1]:.4f}")
    if f[0] - f[1] < 0.002:
        print("LOST OBJECT during lift"); c.close(); sys.exit(3)
    print("[5] transport", place_pre); go(place_pre, 0.08)
    f = c.fingers(); print(f"  gap after transport {f[0]-f[1]:.4f}")
    print("[6] lower", place); go(place, 0.05)
    w2 = c.wrench(); print("  wrench", w2)
    print("[7] release"); c.gripper(OPEN)
    print("[8] retreat", place_pre); go(place_pre, 0.08)
    print("DONE")
    c.close()


if __name__ == "__main__":
    main()
