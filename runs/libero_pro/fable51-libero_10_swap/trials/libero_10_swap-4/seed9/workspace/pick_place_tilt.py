#!/usr/bin/env python3
"""Rim-grasp a mug near the base with a pitched (leaning-in) hand, then place
it flat on a plate.  Same args as pick_place.py plus tilt degrees.

Usage: python3 pick_place_tilt.py <mug_x> <mug_y> <rim_z> <side> <plate_x> <plate_y> <tilt_deg>
"""
import sys
import numpy as np
from ctl import Ctl, DOWN, pitch_down

WALL_R = 0.041
GRASP_BELOW_RIM = 0.03
TABLE_Z = 0.422
PLATE_TOP = 0.456
CARRY_Z = 0.70
OPEN, CLOSED = 0.04, 0.0


def main():
    mx, my, rim_z, side, px, py, tilt = map(float, sys.argv[1:8])
    TILT = pitch_down(np.radians(tilt))
    mug_h = rim_z - TABLE_Z
    grasp = np.array([mx, my + side * WALL_R, rim_z - GRASP_BELOW_RIM])
    pre = grasp.copy(); pre[2] = CARRY_Z
    place = np.array([px, py + side * WALL_R, PLATE_TOP + (mug_h - GRASP_BELOW_RIM) + 0.008])
    place_pre = place.copy(); place_pre[2] = CARRY_Z
    print("grasp", grasp, "place", place, "tilt", tilt)

    c = Ctl()
    print("[0] open gripper"); c.gripper(OPEN)
    print("[1] to pre-grasp (tilted)", pre); c.move_tcp(pre, TILT, n=6, speed=0.1)
    w0 = c.wrench(); print("  wrench", w0)
    print("[2] descend", grasp); c.move_tcp(grasp, TILT, n=5, speed=0.06)
    w1 = c.wrench(); print("  wrench", w1, "delta", None if w0 is None else np.round(w1 - w0, 2))
    pos, _ = c.tcp_world()
    if np.linalg.norm(pos - grasp) > 0.01:
        print("  retrying descent for accuracy"); c.move_tcp(grasp, TILT, n=2, speed=0.03)
    print("[3] close"); f = c.gripper(CLOSED)
    gap = f[0] - f[1]
    print(f"  finger gap {gap:.4f} (0 => closed on air)")
    if gap < 0.002:
        print("GRASP FAILED: nothing held"); c.gripper(OPEN); c.move_tcp(pre, TILT); c.close(); sys.exit(2)
    print("[4] lift", pre); c.move_tcp(pre, TILT, n=4, speed=0.06)
    f = c.fingers(); print(f"  gap after lift {f[0]-f[1]:.4f}")
    if f[0] - f[1] < 0.002:
        print("LOST OBJECT during lift"); c.close(); sys.exit(3)
    print("[5] un-tilt to vertical at carry height")
    c.move_tcp(pre, DOWN, n=4, speed=0.05, min_t=2.0)
    f = c.fingers(); print(f"  gap after untilt {f[0]-f[1]:.4f}")
    print("[6] transport", place_pre); c.move_tcp(place_pre, DOWN, n=8, speed=0.08)
    f = c.fingers(); print(f"  gap after transport {f[0]-f[1]:.4f}")
    print("[7] lower", place); c.move_tcp(place, DOWN, n=4, speed=0.06)
    pos, _ = c.tcp_world()
    if np.linalg.norm(pos - place) > 0.01:
        print("  retrying lower for accuracy"); c.move_tcp(place, DOWN, n=2, speed=0.03)
    w2 = c.wrench(); print("  wrench", w2)
    print("[8] release"); c.gripper(OPEN)
    print("[9] retreat", place_pre); c.move_tcp(place_pre, DOWN, n=3, speed=0.1)
    print("DONE")
    c.close()


if __name__ == "__main__":
    main()
