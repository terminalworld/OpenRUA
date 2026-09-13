#!/usr/bin/env python3
"""Compute a world-frame height map from a camera's depth image.

Usage: python3 scene.py <camera>
Saves <camera>_world.npy: HxWx3 world xyz per pixel, and prints table
height estimate + connected blobs above table.
"""
import sys
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


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
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    from cv_bridge import CvBridge
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(color_msg, "bgr8")
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not tfbuf.can_transform("world", frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T
    np.save(f"{cam}_world.npy", P)
    cv2.imwrite(f"{cam}.png", color)
    z = P[..., 2]
    print("camera pos", T)
    # table height: mode of z in the central region
    zz = z[np.isfinite(z)]
    hist, edges = np.histogram(zz, bins=400)
    table = edges[np.argmax(hist)]
    print(f"dominant z (table?) = {table:.3f}")
    # blobs above table
    mask = (z > table + 0.015) & (z < table + 0.5)
    mask = mask.astype(np.uint8)
    n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 15:
            continue
        m = lab == i
        pts = P[m]
        print(f"blob {i}: px centroid ({cent[i][0]:.0f},{cent[i][1]:.0f}) area {stats[i, cv2.CC_STAT_AREA]} "
              f"world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
              f"ztop {pts[:,2].max():.3f} mean xyz ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f},{pts[:,2].mean():.3f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
