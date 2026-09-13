#!/usr/bin/env python3
"""Grab depth+color+intrinsics+TF for a camera, save a world-frame point
cloud (npz) and print table-plane stats + object clusters above it.

Usage: python3 cloud.py <camera> [zmin_above_table=0.01]
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, mt, timeout=30.0):
    got = {}
    sub = node.create_subscription(mt, topic, lambda m: got.setdefault("m", m), 1)
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


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    z = depth
    X = (u - cx) * z / fx
    Y = (v - cy) * z / fy
    pc = np.stack([X, Y, z], -1) @ R.T + T
    np.savez(f"img/{cam}_cloud.npz", pc=pc, color=color, depth=depth, K=np.array(info.k).reshape(3, 3), R=R, T=T)
    valid = np.isfinite(z) & (z > 0)
    print("cam pos", T, "K", fx, fy, cx, cy, "shape", depth.shape)
    zs = pc[..., 2][valid]
    hist, edges = np.histogram(zs, bins=60)
    top = edges[np.argmax(hist)]
    print("most common z (table?)", round(top, 3), " z range", zs.min().round(3), zs.max().round(3))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
