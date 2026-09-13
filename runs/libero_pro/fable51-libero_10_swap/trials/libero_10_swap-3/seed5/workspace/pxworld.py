#!/usr/bin/env python3
"""World coords for many pixels of one camera in one go.

Usage: python3 pxworld.py <camera> u,v [u,v ...]
Also saves <camera>_depth.npy and prints a per-pixel world xyz.
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    pix = [tuple(int(t) for t in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("pxworld")
    buf = Buffer()
    TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width)
    np.save(f"{cam}_depth.npy", depth)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    np.save(f"{cam}_T.npy", T)
    np.save(f"{cam}_K.npy", np.array(info.k).reshape(3, 3))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    print(f"cam pose world: {T[:3,3]}")
    for (u, v) in pix:
        z = float(depth[v, u])
        if not np.isfinite(z) or z <= 0:
            print(f"({u},{v}): no depth {z}")
            continue
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"({u},{v}) d={z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
