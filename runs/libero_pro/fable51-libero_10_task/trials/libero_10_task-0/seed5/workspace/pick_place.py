#!/usr/bin/env python3
"""Pick an object with a top-down grasp and drop it in the basket.

Usage: python3 -u pick_place.py <x> <y> <grasp_z_tcp> <yaw> [safe_z=0.65]
"""
import sys
import numpy as np
from robot import Robot, TOP_DOWN, yaw_quat, GRIP

BASKET = np.array([-0.005, 0.255, 0.76])  # TCP drop point above basket interior


def main():
    x, y, gz, yaw = map(float, sys.argv[1:5])
    safe_z = float(sys.argv[5]) if len(sys.argv) > 5 else 0.65
    quat = yaw_quat(yaw)
    r = Robot("pick_place")
    print("start tcp", r.tcp_world()[0].round(4), "gap", round(r.finger_gap(), 4), flush=True)

    print("1. open gripper", flush=True)
    r.gripper(GRIP["open_m"])

    print("2. pre-grasp above target", flush=True)
    r.goto((x, y, safe_z), quat, seconds=4.0)

    print("3. descend to grasp height", flush=True)
    r.goto((x, y, gz), quat, seconds=3.0)
    tcp = r.tcp_world()[0]
    if tcp[2] > gz + 0.01:
        print("DESCENT BLOCKED at z=%.4f (fingers probably on top of object); retreating" % tcp[2], flush=True)
        r.goto((x, y, safe_z), quat, seconds=3.0)
        sys.exit(4)

    print("4. close gripper", flush=True)
    gap = r.gripper(GRIP["closed_m"])
    if gap < 0.005:
        print("GRASP FAILED: fingers closed on air (gap=%.4f)" % gap, flush=True)
        r.gripper(GRIP["open_m"])
        r.goto((x, y, safe_z), quat, seconds=3.0)
        sys.exit(2)
    print("grasp evidence: gap=%.4f" % gap, flush=True)

    print("5. lift", flush=True)
    r.goto((x, y, safe_z + 0.05), quat, seconds=3.0)
    gap = r.finger_gap()
    print("gap after lift %.4f" % gap, flush=True)
    if gap < 0.005:
        print("LOST OBJECT during lift", flush=True)
        sys.exit(3)

    print("6. move over basket", flush=True)
    r.goto(tuple(BASKET), TOP_DOWN, seconds=4.0)
    gap = r.finger_gap()
    print("gap over basket %.4f" % gap, flush=True)

    print("7. release", flush=True)
    r.gripper(GRIP["open_m"])

    print("8. retreat up", flush=True)
    r.goto((BASKET[0], BASKET[1], BASKET[2] + 0.08), TOP_DOWN, seconds=3.0)
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
