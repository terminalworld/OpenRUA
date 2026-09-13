#!/usr/bin/env python3
"""Pick the tomato sauce can with a straight (Cartesian-waypoint) descent
from a planar arm configuration, then drop it in the basket."""
import sys
import numpy as np
from robot import Robot, TOP_DOWN, GRIP

X, Y = -0.114, 0.073
GZ = 0.475           # TCP grasp height (can mid-height; can spans 0.425..0.52)
HIGH = 0.80
NICE_SEED = [0.18, 0.3, 0.0, -2.4, 0.0, 2.7, 0.97]
BASKET = np.array([-0.005, 0.255, 0.76])


def chain(r, xy, zs, seed):
    qs = []
    for z in zs:
        seed = r.solve_ik((xy[0], xy[1], z), TOP_DOWN, seed=seed)
        qs.append(seed)
    return qs


def main():
    r = Robot("can_pick")
    print("start tcp", r.tcp_world()[0].round(4), "gap", round(r.finger_gap(), 4), flush=True)
    r.gripper(GRIP["open_m"])

    print("1. up to HIGH in current config", flush=True)
    r.goto((X, Y, HIGH), TOP_DOWN, seconds=3.0)

    print("2. reconfigure to planar config at HIGH", flush=True)
    q_high = r.solve_ik((X, Y, HIGH), TOP_DOWN, seed=NICE_SEED)
    print("   q_high", np.round(q_high, 3), flush=True)
    r.move(q_high, seconds=4.0)
    print("   tcp", r.tcp_world()[0].round(4), flush=True)

    print("3. straight descent to grasp height", flush=True)
    zs = [0.70, 0.65, 0.60, 0.55, 0.52, GZ]
    qs = chain(r, (X, Y), zs, q_high)
    r.move(qs[-1], seconds=6.0, waypoints=qs[:-1])
    tcp = r.tcp_world()[0]
    print("   tcp", tcp.round(4), flush=True)
    if tcp[2] > GZ + 0.01 or abs(tcp[0] - X) > 0.01 or abs(tcp[1] - Y) > 0.01:
        print("DESCENT OFF TARGET; retreating", flush=True)
        r.move(q_high, seconds=4.0, waypoints=qs[::-1][1:])
        sys.exit(4)

    print("4. close gripper", flush=True)
    gap = r.gripper(GRIP["closed_m"])
    if gap < 0.01:
        print("GRASP FAILED gap=%.4f" % gap, flush=True)
        r.gripper(GRIP["open_m"])
        r.move(q_high, seconds=4.0, waypoints=qs[::-1][1:])
        sys.exit(2)
    print("grasp evidence gap=%.4f" % gap, flush=True)

    print("5. straight lift", flush=True)
    up = qs[::-1][1:]  # back up through the same waypoints
    r.move(up[-1], seconds=5.0, waypoints=up[:-1])
    print("   tcp", r.tcp_world()[0].round(4), "gap", round(r.finger_gap(), 4), flush=True)
    if r.finger_gap() < 0.01:
        print("LOST OBJECT", flush=True)
        sys.exit(3)

    print("6. over basket", flush=True)
    r.goto(tuple(BASKET), TOP_DOWN, seconds=4.0)
    print("   gap", round(r.finger_gap(), 4), flush=True)

    print("7. release", flush=True)
    r.gripper(GRIP["open_m"])

    print("8. retreat", flush=True)
    r.goto((BASKET[0], BASKET[1], BASKET[2] + 0.08), TOP_DOWN, seconds=3.0)
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
