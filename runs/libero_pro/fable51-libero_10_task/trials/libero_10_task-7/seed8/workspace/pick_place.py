#!/usr/bin/env python3
"""Pick an object at (x,y) with a top-down grasp and drop it into the basket.

Usage: python3 -u pick_place.py <name> <x> <y> <grasp_tcp_z> [pre_z]
"""
import sys
import numpy as np
from ctl import Ctl, Q_HAND_X

BASKET = np.array([0.0, 0.25])
Z_TRANSPORT = 0.76   # TCP height while carrying (bottle bottom clears rim 0.63)
Z_RELEASE = 0.68     # TCP height when opening over the basket
OPEN = 0.04


def main():
    name, x, y, zg = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    zpre = float(sys.argv[5]) if len(sys.argv) > 5 else 0.62
    q = Q_HAND_X
    c = Ctl()
    print(f"[{name}] start tcp={np.round(c.tcp_pose()[0],3)} fingers={np.round(c.finger_gap(),4)}")

    print(f"[{name}] open gripper"); c.gripper(OPEN)
    print(f"[{name}] pre-grasp above object"); c.move_tcp((x, y, zpre), q, 4.0)
    print(f"[{name}] descend"); c.move_tcp((x, y, zg), q, 2.5)
    print(f"[{name}] close"); f1, f2 = c.gripper(0.0)
    gap = abs(f1) + abs(f2)
    print(f"[{name}] finger gap after close = {gap:.4f} m ({'HOLDING' if gap > 0.008 else 'EMPTY?'})")
    print(f"[{name}] lift"); c.move_tcp((x, y, zpre), q, 2.5)
    print(f"[{name}] lift check fingers={np.round(c.finger_gap(),4)}")
    print(f"[{name}] lift to transport height"); c.move_tcp((x, y, Z_TRANSPORT), q, 2.0)
    print(f"[{name}] move over basket"); c.move_tcp((BASKET[0], BASKET[1], Z_TRANSPORT), q, 4.0)
    print(f"[{name}] check fingers={np.round(c.finger_gap(),4)}")
    print(f"[{name}] lower over basket"); c.move_tcp((BASKET[0], BASKET[1], Z_RELEASE), q, 2.0)
    print(f"[{name}] release"); c.gripper(OPEN)
    print(f"[{name}] retreat up"); c.move_tcp((BASKET[0], BASKET[1], Z_TRANSPORT), q, 2.0)
    print(f"[{name}] DONE tcp={np.round(c.tcp_pose()[0],3)}")


if __name__ == "__main__":
    main()
