#!/usr/bin/env python3
"""World coordinates for many pixels from one camera's current depth frame.

Usage: python3 px_batch.py <camera> u1,v1 u2,v2 ...
Also saves <camera>_cloud.npz with the full HxWx3 world-xyz array.
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=20.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
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
    pixels = [tuple(int(t) for t in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("px_batch")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    H, W = depth_msg.height, depth_msg.width
    depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(H, W)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y,
                  t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(us - cx) * depth / fx, (vs - cy) * depth / fy, depth], -1)
    world = pc @ R.T + T
    np.savez(f"{cam}_cloud.npz", world=world, depth=depth)
    for (u, v) in pixels:
        x, y, z = world[v, u]
        print(f"px ({u},{v}) depth={depth[v,u]:.4f} -> world {x:.4f} {y:.4f} {z:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
