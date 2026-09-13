#!/usr/bin/env python3
"""Grab depth+info+TF for a camera; convert pixels to world, save arrays.
Usage: python3 scene.py <camera> [u v]...
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, mt, timeout=20.0):
    got = {}
    sub = node.create_subscription(mt, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    px = [(int(sys.argv[i]), int(sys.argv[i + 1])) for i in range(2, len(sys.argv) - 1, 2)]
    rclpy.init()
    node = rclpy.create_node("scene")
    buf = Buffer(); TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
    color = CvBridge().imgmsg_to_cv2(color_msg, "bgr8")
    np.save(f"{cam}_depth.npy", depth)
    cv2.imwrite(f"{cam}.png", color)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4); T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    np.save(f"{cam}_T.npy", T)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    np.save(f"{cam}_K.npy", np.array([fx, fy, cx, cy]))
    print("cam pos", T[:3, 3], "K", fx, fy, cx, cy, "size", depth.shape)
    for (u, v) in px:
        z = depth[v, u]
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"px({u},{v}) depth={z:.3f} world=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
