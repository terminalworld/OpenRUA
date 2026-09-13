#!/usr/bin/env python3
"""Grab color+depth+info from a camera, cache world-point cloud as .npz.
Usage: python3 scan.py <camera>
Then query pixels: python3 scan.py <camera> u v [u v ...]
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2


def grab(node, topic, mt, timeout=30.0):
    got = {}
    sub = node.create_subscription(mt, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    if len(sys.argv) > 2:
        d = np.load(f"{cam}_cloud.npz")
        P = d["P"]
        args = list(map(int, sys.argv[2:]))
        for u, v in zip(args[::2], args[1::2]):
            print(f"({u},{v}) -> {P[v, u]}")
        return
    rclpy.init()
    node = rclpy.create_node("scan")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(color_msg, "bgr8")
    cv2.imwrite(f"{cam}.png", color)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    vs, us = np.mgrid[0:h, 0:w]
    X = (us - cx) * depth / fx
    Y = (vs - cy) * depth / fy
    pc = np.stack([X, Y, depth], -1)
    P = pc @ R.T + T
    np.savez(f"{cam}_cloud.npz", P=P, depth=depth)
    print(f"saved {cam}_cloud.npz  shape={P.shape}  z range {np.nanmin(P[...,2]):.3f}..{np.nanmax(P[...,2]):.3f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
