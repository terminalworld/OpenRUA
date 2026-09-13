#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for cameras, save world-frame point clouds.

Usage: python3 scene.py <cam> [<cam> ...]
Writes <cam>_rgb.png, <cam>_xyz.npy (HxWx3 world coords, nan where invalid).
Also prints all TF frames known and world->hand pose if available.
"""
import sys
import numpy as np
import rclpy
from rclpy.time import Time
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, typ, timeout=30.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
    return got["m"]


def T_of(t):
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
    cams = sys.argv[1:]
    rclpy.init()
    node = rclpy.create_node("scene")
    buf = Buffer()
    TransformListener(buf, node)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.2)
    br = CvBridge()
    for cam in cams:
        rgb = grab(node, f"/{cam}/color/image_raw", Image)
        dep = grab(node, f"/{cam}/depth/image_raw", Image)
        info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
        img = br.imgmsg_to_cv2(rgb, "bgr8")
        d = br.imgmsg_to_cv2(dep, "passthrough").astype(np.float64)
        frame = f"{cam}_optical_frame"
        end = node.get_clock().now().nanoseconds / 1e9 + 10
        while node.get_clock().now().nanoseconds / 1e9 < end:
            rclpy.spin_once(node, timeout_sec=0.2)
            if buf.can_transform("world", frame, Time()):
                break
        t = buf.lookup_transform("world", frame, Time())
        T = T_of(t)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        H, W = d.shape
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        X = (u - cx) * d / fx
        Y = (v - cy) * d / fy
        pts = np.stack([X, Y, d, np.ones_like(d)], -1) @ T.T
        xyz = pts[..., :3]
        bad = ~np.isfinite(d) | (d <= 0)
        xyz[bad] = np.nan
        cv2.imwrite(f"{cam}_rgb.png", img)
        np.save(f"{cam}_xyz.npy", xyz)
        print(cam, "cam pos", np.round(T[:3, 3], 3), "depth range",
              np.nanmin(d[~bad]) if (~bad).any() else None, np.nanmax(d[~bad]) if (~bad).any() else None)
    print(buf.all_frames_as_string())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
