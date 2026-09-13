#!/usr/bin/env python3
"""Scene helper: dump TF frames and convert pixels of a camera to world.

Usage: python3 scene.py <camera> u,v [u,v ...]
"""
import struct
import sys

import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    pix = [tuple(int(a) for a in p.split(",")) for p in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("scene")
    buf = Buffer()
    TransformListener(buf, node)
    import time
    t0 = time.time()
    while time.time() - t0 < 3:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(buf.all_frames_as_string())
    for tgt in ["panda_link0", "panda_hand", f"{cam}_optical_frame"]:
        try:
            t = buf.lookup_transform("world", tgt, Time())
            tr, q = t.transform.translation, t.transform.rotation
            print(f"world->{tgt}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
        except Exception as e:
            print(f"world->{tgt}: FAIL {e}")
    if not pix:
        return
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    t = buf.lookup_transform("world", f"{cam}_optical_frame", Time())
    T = np.eye(4)
    T[:3, :3] = quat_R(t.transform.rotation)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    d = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    for u, v in pix:
        z = float(d[v, u])
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"px({u},{v}) depth={z:.4f} -> world ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
