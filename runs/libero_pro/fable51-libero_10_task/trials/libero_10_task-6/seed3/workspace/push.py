#!/usr/bin/env python3
"""Straight push with the closed gripper, hand vertical.

python3 push.py sx sy ex ey z yaw_deg [hover_z]
yaw_deg: direction (world, from +x) along which the fingers close (= palm long axis).
"""
import sys
import numpy as np, rclpy
from arm import Arm, DOWN_X


def qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


def down_yaw(yaw_deg, base=DOWN_X):
    """Hand pointing down, fingers closing along world direction yaw_deg (base=DOWN_X)."""
    h = np.radians(yaw_deg) / 2
    return qmul(np.array([0, 0, np.sin(h), np.cos(h)]), base)


def main():
    sx, sy, ex, ey, z, yaw = map(float, sys.argv[1:7])
    hover = float(sys.argv[7]) if len(sys.argv) > 7 else 0.60
    Q = down_yaw(yaw)
    a = Arm()
    a.gripper(0.0)
    print("== hover above start")
    a.goto([sx, sy, hover], Q, seconds=4.0, max_jump=2.5)  # free-space move, larger jump OK
    print("== descend")
    a.goto([sx, sy, z], Q, seconds=3.0)
    n = max(1, int(np.hypot(ex - sx, ey - sy) / 0.035 + 0.999))
    for i in range(1, n + 1):
        p = [sx + (ex - sx) * i / n, sy + (ey - sy) * i / n, z]
        print(f"== push step {i}/{n} -> {np.round(p, 3)}")
        a.goto(p, Q, seconds=2.5)
    print("== lift")
    a.goto([ex, ey, hover], Q, seconds=3.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
