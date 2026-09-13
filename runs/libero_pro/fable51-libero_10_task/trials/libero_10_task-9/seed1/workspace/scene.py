#!/usr/bin/env python3
"""Grab color+depth+info+TF for cameras, save world-frame point clouds.

Usage: python3 scene.py <cam> [<cam> ...]
Writes snaps/<cam>_rgb.png, snaps/<cam>_xyz.npy (HxWx3 world coords, nan
where invalid) and prints the world->panda_link0 transform.
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


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tf_matrix(tfbuf, node, parent, child):
    import time
    end = time.time() + 15
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform(parent, child, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform(parent, child, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def main():
    cams = sys.argv[1:]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    bridge = CvBridge()
    T0 = tf_matrix(tfbuf, node, "world", "panda_link0")
    print("world->panda_link0 translation:", T0[:3, 3], "\nR:\n", T0[:3, :3])
    np.save("snaps/T_world_link0.npy", T0)
    for cam in cams:
        rgb = bridge.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
        depth = bridge.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
        info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        T = tf_matrix(tfbuf, node, "world", f"{cam}_optical_frame")
        h, w = depth.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        z = depth
        valid = np.isfinite(z) & (z > 0)
        X = (u - cx) * z / fx
        Y = (v - cy) * z / fy
        pts = np.stack([X, Y, z, np.ones_like(z)], -1) @ T.T
        xyz = pts[..., :3]
        xyz[~valid] = np.nan
        cv2.imwrite(f"snaps/{cam}_rgb.png", rgb)
        np.save(f"snaps/{cam}_xyz.npy", xyz)
        print(cam, "cam pos", T[:3, 3], "depth range", np.nanmin(z[valid]), np.nanmax(z[valid]))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
