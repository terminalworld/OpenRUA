#!/usr/bin/env python3
"""Rim-grasp a mug and place it on a plate.

Usage: python3 pick_place.py <mug_x> <mug_y> <rim_z> <side:+1|-1> <plate_x> <plate_y>
side: which y-side of the mug to grasp (+1 = +y side, -1 = -y side; pick the
side away from the handle). Grasp point is on the wall at radius WALL_R.
"""
import sys
import numpy as np
from ctl import Ctl, DOWN

WALL_R = 0.041       # radius of the mug wall centre
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
    # when placed, TCP sits (mug_h - GRASP_BELOW_RIM) above the mug bottom
    place = np.array([px, py + side * WALL_R, PLATE_TOP + (mug_h - GRASP_BELOW_RIM) + 0.008])
    place_pre = place.copy(); place_pre[2] = CARRY_Z
    print("grasp", grasp, "place", place)

    c = Ctl()
    print("[0] open gripper"); c.gripper(OPEN)
    print("[1] to pre-grasp", pre); c.move_tcp(pre, DOWN, n=6, speed=0.1)
    w0 = c.wrench(); print("  wrench", w0)
    print("[2] descend", grasp); c.move_tcp(grasp, DOWN, n=4, speed=0.08)
    w1 = c.wrench(); print("  wrench", w1, "delta", None if w0 is None else np.round(w1 - w0, 2))
    print("[3] close"); f = c.gripper(CLOSED)
    gap = f[0] - f[1]
    print(f"  finger gap {gap:.4f} (0 => closed on air)")
    if gap < 0.002:
        print("GRASP FAILED: nothing held"); c.gripper(OPEN); c.move_tcp(pre, DOWN); c.close(); sys.exit(2)
    print("[4] lift", pre); c.move_tcp(pre, DOWN, n=4, speed=0.08)
    f = c.fingers(); print(f"  gap after lift {f[0]-f[1]:.4f}")
    print("[5] transport", place_pre); c.move_tcp(place_pre, DOWN, n=8, speed=0.08)
    f = c.fingers(); print(f"  gap after transport {f[0]-f[1]:.4f}")
    print("[6] lower", place); c.move_tcp(place, DOWN, n=4, speed=0.08)
    w2 = c.wrench(); print("  wrench", w2)
    print("[7] release"); c.gripper(OPEN)
    print("[8] retreat", place_pre); c.move_tcp(place_pre, DOWN, n=3, speed=0.1)
    print("DONE")
    c.close()


if __name__ == "__main__":
    main()
