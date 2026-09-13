#!/usr/bin/env python3
"""Grab color, depth, intrinsics and world<-optical TF for cameras; save npz.
Usage: python3 grab.py cam1 [cam2 ...]
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


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


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = rclpy.create_node("grab")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    import time
    for cam in sys.argv[1:]:
        color = grab(node, f"/{cam}/color/image_raw", Image)
        depth = grab(node, f"/{cam}/depth/image_raw", Image)
        info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
        img = br.imgmsg_to_cv2(color, "bgr8")
        dep = br.imgmsg_to_cv2(depth, "passthrough").astype(np.float32)
        frame = f"{cam}_optical_frame"
        t0 = time.time()
        while time.time() - t0 < 10:
            rclpy.spin_once(node, timeout_sec=0.2)
            if tfbuf.can_transform("world", frame, rclpy.time.Time()):
                break
        t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
        T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        K = np.array(info.k).reshape(3, 3)
        np.savez(f"{cam}.npz", img=img, depth=dep, K=K, T=T)
        cv2.imwrite(f"{cam}.png", img)
        print(cam, img.shape, dep.shape, "T=", T[:3, 3])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
