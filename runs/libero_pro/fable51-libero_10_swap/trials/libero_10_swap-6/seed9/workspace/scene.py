#!/usr/bin/env python3
"""Grab color+depth+info from a camera, save world-frame point cloud + image.

Usage: python3 scene.py <camera>
Writes <camera>_pts.npy (H,W,3 world xyz, nan where invalid) and <camera>.png.
"""
import sys
import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
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
    rclpy.init()
    node = rclpy.create_node("scene_" + cam)
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    color = grab(node, f"/{cam}/color/image_raw", Image)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    img = br.imgmsg_to_cv2(color, "bgr8")
    d = br.imgmsg_to_cv2(depth, "passthrough").astype(np.float64)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = d.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * d / fx
    Y = (v - cy) * d / fy
    pc = np.stack([X, Y, d], -1)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    pw = pc @ R.T + T
    bad = ~np.isfinite(d) | (d <= 0)
    pw[bad] = np.nan
    np.save(f"{cam}_pts.npy", pw)
    cv2.imwrite(f"{cam}.png", img)
    print(cam, "cam pos", T, "saved")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
