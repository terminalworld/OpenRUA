#!/usr/bin/env python3
"""Convert several pixels of one camera to world coords in one go.

Usage: python3 locate.py <camera> u,v[,label] [u,v[,label] ...]
Also saves <camera>.png and <camera>_depth.npy for offline inspection.
"""
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, msg_type, timeout=15.0):
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


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    pix = []
    for a in sys.argv[2:]:
        parts = a.split(",")
        pix.append((int(parts[0]), int(parts[1]),
                    parts[2] if len(parts) > 2 else ""))
    rclpy.init()
    node = rclpy.create_node("locate")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    bridge = CvBridge()
    color = grab(node, f"/{cam}/color/image_raw", Image)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    img = bridge.imgmsg_to_cv2(color, "bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
    cv2.imwrite(f"{cam}.png", img)
    np.save(f"{cam}_depth.npy", depth)

    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z]
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    print(f"camera {cam} at world {T[:3, 3].round(3)}")
    for u, v, label in pix:
        z = float(depth[v, u])
        if not np.isfinite(z) or z <= 0:
            print(f"{label:>12s} ({u},{v}): no depth")
            continue
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"{label:>12s} ({u},{v}) d={z:.3f} -> "
              f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
