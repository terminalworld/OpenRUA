#!/usr/bin/env python3
"""Segment objects above the table from a camera's depth + color and report
world-frame centroid / bbox / top height / mean colour for each blob.

Usage: python3 scene.py [camera=birdview] [zmin=0.44]
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
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.44
    rclpy.init()
    node = rclpy.create_node("scene")
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
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T  # world coords (h,w,3)
    np.save(f"{cam}_world.npy", P)
    cv2.imwrite(f"{cam}.png", color)
    valid = np.isfinite(depth) & (depth > 0)
    mask = valid & (P[..., 2] > zmin) & (P[..., 2] < 1.2)
    n, lab = cv2.connectedComponents(mask.astype(np.uint8))
    print(f"camera {cam}: {n-1} blobs with z>{zmin}")
    for i in range(1, n):
        m = lab == i
        if m.sum() < 15:
            continue
        pts = P[m]
        col = color[m].mean(0)[::-1]  # rgb
        vs, us = np.nonzero(m)
        print(f"blob {i}: n={m.sum()} px(u={us.min()}-{us.max()}, v={vs.min()}-{vs.max()}) "
              f"centroid=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) "
              f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
              f"ztop={pts[:,2].max():.3f} zmin={pts[:,2].min():.3f} rgb={col.astype(int)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
