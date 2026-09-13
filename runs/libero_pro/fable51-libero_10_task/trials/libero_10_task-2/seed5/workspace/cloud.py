#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera; save world-frame point cloud
as <cam>_cloud.npy (H x W x 3) plus <cam>.png color."""
import struct
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    import time
    t0 = time.time()
    while "m" not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("cloud")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    color = grab(node, f"/{cam}/color/image_raw", Image)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    img = br.imgmsg_to_cv2(color, "bgr8")
    d = br.imgmsg_to_cv2(depth, "passthrough").astype(np.float64)
    cv2.imwrite(f"{cam}.png", img)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d, np.ones_like(d)], -1)
    frame = f"{cam}_optical_frame"
    import time
    t0 = time.time()
    while time.time() - t0 < 15 and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    pw = (pc.reshape(-1, 4) @ T.T)[:, :3].reshape(H, W, 3)
    np.save(f"{cam}_cloud.npy", pw)
    print(f"{cam}: {W}x{H} cloud saved; cam pos {np.round(T[:3,3],3)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
