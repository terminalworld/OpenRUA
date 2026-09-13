#!/usr/bin/env python3
"""Grab color+depth+info+TF for a camera; convert pixels to world.

Usage: python3 scene.py <camera> u,v [u,v ...]
Also saves <camera>_depth.npy and <camera>.png.
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, typ):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
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
    pix = [tuple(int(t) for t in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("scene")
    buf = Buffer(); TransformListener(buf, node)
    br = CvBridge()
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    cv2.imwrite(f"{cam}.png", color)
    np.save(f"{cam}_depth.npy", depth)
    frame = f"{cam}_optical_frame"
    while not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4); T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    np.save(f"{cam}_T.npy", T)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    print("depth shape", depth.shape, "finite range",
          np.nanmin(depth[np.isfinite(depth)]), np.nanmax(depth[np.isfinite(depth)]))
    for (u, v) in pix:
        z = float(depth[v, u])
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"px({u},{v}) depth={z:.4f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
