#!/usr/bin/env python3
"""Segment objects above the table from a camera's depth frame and print
their world-frame bounding boxes. Usage: python3 scan.py <camera> [zmin]
"""
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scipy import ndimage


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
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.435
    rclpy.init()
    node = rclpy.create_node("scan")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(color_msg, "bgr8")
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    vs, us = np.mgrid[0:h, 0:w]
    X = (us - cx) * depth / fx
    Y = (vs - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T
    valid = np.isfinite(depth) & (depth > 0)
    mask = valid & (P[..., 2] > zmin) & (P[..., 2] < 0.75)
    lab, n = ndimage.label(mask)
    print(f"camera at {T}, {n} blobs")
    for i in range(1, n + 1):
        m = lab == i
        if m.sum() < 40:
            continue
        pts = P[m]
        col = color[m].mean(0)[::-1].astype(int)
        vv, uu = np.nonzero(m)
        xy = pts[:, :2] - pts[:, :2].mean(0); ev, evec = np.linalg.eigh(xy.T @ xy); ax = evec[:, 1]; yaw = np.degrees(np.arctan2(ax[1], ax[0]))
        print(f"blob {i}: yaw {yaw:.1f}deg px {m.sum()} u[{uu.min()},{uu.max()}] v[{vv.min()},{vv.max()}] "
              f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
              f"z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] centre ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) rgb {col}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
