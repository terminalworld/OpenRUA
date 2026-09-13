#!/usr/bin/env python3
"""Intersect a camera pixel ray with a world z-plane.

Usage: python3 px_at_z.py <camera> <z> <u> <v> [<u> <v> ...]
Prints "u v -> x y z" for each pixel. Uses intrinsics + TF only (no depth),
so it is exact for points known to lie at height z (e.g. a mug rim).
"""
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam, zp = sys.argv[1], float(sys.argv[2])
    pix = [(int(sys.argv[i]), int(sys.argv[i + 1])) for i in range(3, len(sys.argv), 2)]
    rclpy.init()
    node = rclpy.create_node("px_at_z")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    got = {}
    node.create_subscription(CameraInfo, f"/{cam}/color/camera_info",
                             lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    info = got["m"]
    frame = f"{cam}_optical_frame"
    while not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    o = np.array([t.transform.translation.x, t.transform.translation.y,
                  t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    for u, v in pix:
        d_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        d = R @ d_cam
        s = (zp - o[2]) / d[2]
        p = o + s * d
        print(f"{u} {v} -> {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
