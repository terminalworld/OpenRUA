#!/usr/bin/env python3
"""Cache one camera's color, depth, intrinsics and world<-optical TF to
<cam>_cache.npz so pixels can be converted offline many times.

Usage: python3 cam_cache.py <cam> [<cam> ...]
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_T(t):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z]
    return T


def main():
    rclpy.init()
    node = rclpy.create_node("cam_cache")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    import time
    for cam in sys.argv[1:]:
        color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
        depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
        info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
        frame = f"{cam}_optical_frame"
        end = time.time() + 15
        while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
            rclpy.spin_once(node, timeout_sec=0.2)
        T = quat_T(tfbuf.lookup_transform("world", frame, rclpy.time.Time()))
        np.savez(f"{cam}_cache.npz", color=color, depth=depth, K=np.array(info.k).reshape(3, 3), T=T)
        cv2.imwrite(f"{cam}.png", color)
        print(cam, depth.shape, "K=", np.array(info.k).reshape(3, 3)[0], "T=", T[:3, 3])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
