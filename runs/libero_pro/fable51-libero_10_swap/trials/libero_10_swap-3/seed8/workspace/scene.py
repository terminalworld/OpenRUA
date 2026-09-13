#!/usr/bin/env python3
"""Dump world-frame point clouds / lookups for a camera.

Usage:
  python3 scene.py <cam> tf                 # print world->camera TF & intrinsics
  python3 scene.py <cam> px u v [u v ...]   # world xyz of pixels
  python3 scene.py <cam> save               # save <cam>_xyz.npy (HxWx3 world coords)
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


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


def main():
    cam, mode = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype(np.float64)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    if mode == "tf":
        print("T", T, "\nR", R, "\nK", fx, fy, cx, cy, "size", W, H)
        print("depth range", np.nanmin(depth), np.nanmax(depth))
    elif mode == "px":
        vals = list(map(int, sys.argv[3:]))
        for u, v in zip(vals[::2], vals[1::2]):
            z = depth[v, u]
            p = R @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z]) + T
            print(f"({u},{v}) depth={z:.4f} world= {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    elif mode == "save":
        us, vs = np.meshgrid(np.arange(W), np.arange(H))
        pc = np.stack([(us - cx) * depth / fx, (vs - cy) * depth / fy, depth], -1)
        world = pc @ R.T + T
        np.save(f"{cam}_xyz.npy", world)
        print("saved", f"{cam}_xyz.npy", world.shape)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
