#!/usr/bin/env python3
"""Segment objects above the table in a top-down camera and print world centroids.

Usage: python3 scan.py [camera=birdview]
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


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
    cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"
    rclpy.init()
    node = rclpy.create_node("scan")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    while not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - cx) * depth / fx
    Y = (vs - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T  # world coords per pixel
    Z = P[..., 2]
    valid = np.isfinite(depth) & (depth > 0)
    table_z = 0.426
    mask = valid & (Z > table_z + 0.015) & (Z < table_z + 0.5)
    # exclude the robot: approx region near base
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 15:
            continue
        m = lab == i
        pts = P[m]
        col = color[m].mean(0)[::-1]
        u, v = cents[i]
        print(f"comp {i}: px=({u:.0f},{v:.0f}) area={area} "
              f"world centroid=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) "
              f"zmax={pts[:,2].max():.3f} zmin={pts[:,2].min():.3f} "
              f"xrange=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yrange=({pts[:,1].min():.3f},{pts[:,1].max():.3f}) rgb={col.astype(int)}")
    # plates: slightly above table
    pm = valid & (Z > table_z + 0.003) & (Z <= table_z + 0.015)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(pm.astype(np.uint8), 8)
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 30:
            continue
        m = lab == i
        pts = P[m]
        u, v = cents[i]
        print(f"low comp {i}: px=({u:.0f},{v:.0f}) area={area} "
              f"world centroid=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) zmax={pts[:,2].max():.3f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
