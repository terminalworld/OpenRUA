#!/usr/bin/env python3
"""Grab color+depth+info+TF for a camera and back-project pixels to world.

Usage: python3 scene.py <camera> [u v]...   -> prints world xyz per pixel
       also saves <camera>_pts.npy (HxWx3 world coords) for offline use.
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, typ, timeout=20.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def cloud(cam):
    node = rclpy.create_node("scene_" + cam)
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    color = grab(node, f"/{cam}/color/image_raw", Image)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    img = br.imgmsg_to_cv2(color, "bgr8")
    d = br.imgmsg_to_cv2(depth, "passthrough").astype(np.float64)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time()).transform
    R = quat_R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    T = np.array([t.translation.x, t.translation.y, t.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = d.shape
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    pc = np.stack([(us - cx) * d / fx, (vs - cy) * d / fy, d], -1)
    pw = pc @ R.T + T
    node.destroy_node()
    return img, d, pw


if __name__ == "__main__":
    cam = sys.argv[1]
    rclpy.init()
    img, d, pw = cloud(cam)
    cv2.imwrite(f"{cam}.png", img)
    np.save(f"{cam}_pts.npy", pw)
    px = [int(x) for x in sys.argv[2:]]
    for u, v in zip(px[::2], px[1::2]):
        print(u, v, "->", np.round(pw[v, u], 4), "depth", round(d[v, u], 4))
    rclpy.shutdown()
