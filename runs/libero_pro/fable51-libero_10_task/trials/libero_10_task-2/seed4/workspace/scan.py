#!/usr/bin/env python3
"""Top-down scene scan from a camera: world-frame point cloud -> objects.

Usage: python3 scan.py <camera> [zmin=0.905]
Prints connected blobs of points above the table (world z > zmin) with
their centroid, bbox and max height. Also saves <camera>_pts.npy (HxWx3).
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scipy import ndimage


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
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
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.905
    rclpy.init()
    node = rclpy.create_node("scan")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    H, W = depth.height, depth.width
    d = np.frombuffer(depth.data, dtype=np.float32).reshape(H, W)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], -1)
    pw = pc @ R.T + T
    np.save(f"{cam}_pts.npy", pw)
    ok = np.isfinite(d) & (d > 0)
    above = ok & (pw[..., 2] > zmin)
    lab, n = ndimage.label(above)
    print(f"{cam}: {n} blobs above z={zmin}")
    for i in range(1, n + 1):
        m = lab == i
        if m.sum() < 15:
            continue
        P = pw[m]
        vs, us = np.where(m)
        print(f" blob{i}: n={m.sum():5d} px(u{us.min()}-{us.max()},v{vs.min()}-{vs.max()}) "
              f"centroid=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) "
              f"x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] "
              f"z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
