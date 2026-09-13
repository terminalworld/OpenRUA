#!/usr/bin/env python3
"""Dump a camera's depth frame as a world-frame point cloud + color image.

Usage: python3 scene.py <camera>
Saves snaps/<camera>_pts.npy (H,W,3 world xyz) and snaps/<camera>.png,
and prints the world->panda_link0 transform.
"""
import sys
import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=20.0):
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


def tf_mat(t):
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
    return T


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene")
    buf = Buffer()
    TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(color_msg, "bgr8")
    import time
    end = time.time() + 15
    frame = f"{cam}_optical_frame"
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buf.can_transform("world", frame, rclpy.time.Time()) and \
           buf.can_transform("world", "panda_link0", rclpy.time.Time()):
            break
    T = tf_mat(buf.lookup_transform("world", frame, rclpy.time.Time()))
    Tb = tf_mat(buf.lookup_transform("world", "panda_link0", rclpy.time.Time()))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth, np.ones_like(depth)], -1)
    Pw = P @ T.T
    np.save(f"snaps/{cam}_pts.npy", Pw[..., :3])
    cv2.imwrite(f"snaps/{cam}.png", color)
    print("cam T:\n", np.round(T, 4))
    print("world->panda_link0:\n", np.round(Tb, 4))
    print("depth range", np.nanmin(depth), np.nanmax(depth))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
