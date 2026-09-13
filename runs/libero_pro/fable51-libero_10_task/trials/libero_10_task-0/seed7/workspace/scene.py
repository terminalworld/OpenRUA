#!/usr/bin/env python3
"""Segment objects above the table from the birdview depth camera.

Usage: python3 scene.py [zmin=0.445]
Prints clusters: centroid world x y, top z, pixel bbox, pixel count.
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, msg_type, timeout=60.0):
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
    zmin = float(sys.argv[1]) if len(sys.argv) > 1 else 0.445
    cam = sys.argv[2] if len(sys.argv) > 2 else "birdview"
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    import time
    end = time.time() + 20
    frame = f"{cam}_optical_frame"
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])

    H, W = depth_msg.height, depth_msg.width
    depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(H, W)
    color = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(H, W, -1)[:, :, :3]
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - cx) * depth / fx
    Y = (vs - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T
    Z = P[..., 2]
    mask = (Z > zmin) & (Z < 0.9) & np.isfinite(depth)
    # exclude the robot: it is a big blob; we'll report all and let the user judge
    n, lab = cv2.connectedComponents(mask.astype(np.uint8))
    out = []
    for i in range(1, n):
        m = lab == i
        cnt = m.sum()
        if cnt < 8:
            continue
        pts = P[m]
        vv, uu = np.where(m)
        top = pts[:, 2].max()
        # centroid of the top-most part (within 2cm of top) is a better XY for grasping
        topm = pts[:, 2] > top - 0.03
        c = pts[topm].mean(0)
        col = color[m].mean(0)
        out.append((cnt, c, top, uu.min(), uu.max(), vv.min(), vv.max(), col))
    out.sort(key=lambda o: -o[0])
    for cnt, c, top, u0, u1, v0, v1, col in out:
        print(f"px={cnt:5d} xy=({c[0]:+.3f},{c[1]:+.3f}) top_z={top:.3f} "
              f"bbox u[{u0}-{u1}] v[{v0}-{v1}] rgb={col.astype(int)}")
    np.save(f"{cam}_P.npy", P)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
