#!/usr/bin/env python3
"""Convert one image pixel to world coordinates (depth + intrinsics + TF).

Usage: python3 tools/perception/px2world.py <camera> <u> <v>
Prints "x y z" in the world frame for pixel column u, row v of the
camera's CURRENT depth frame. Exits nonzero if the pixel has no valid
depth. Pure geometry; which pixel is worth asking about is your call.
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def _grab(node, topic, msg_type, timeout=15.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic} within {timeout}s")
    return got["m"]


def main() -> None:
    if len(sys.argv) != 4:
        raise SystemExit(__doc__)
    cam, u, v = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    rclpy.init()
    node = rclpy.create_node("px2world")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)

    depth = _grab(node, f"/{cam}/depth/image_raw", Image)
    info = _grab(node, f"/{cam}/color/camera_info", CameraInfo)
    if not (0 <= v < depth.height and 0 <= u < depth.width):
        raise SystemExit(f"pixel ({u},{v}) outside {depth.width}x{depth.height}")
    z = struct.unpack_from("<f", depth.data, (v * depth.width + u) * 4)[0]
    if not np.isfinite(z) or z <= 0:
        raise SystemExit(f"no valid depth at ({u},{v}): {z}")

    fx, fy = info.k[0], info.k[4]
    cx, cy = info.k[2], info.k[5]
    p_cam = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])

    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    x, y, zz, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + zz * zz), 2 * (x * y - zz * w), 2 * (x * zz + y * w)],
        [2 * (x * y + zz * w), 1 - 2 * (x * x + zz * zz), 2 * (y * zz - x * w)],
        [2 * (x * zz - y * w), 2 * (y * zz + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z]
    p = T @ p_cam
    print(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
