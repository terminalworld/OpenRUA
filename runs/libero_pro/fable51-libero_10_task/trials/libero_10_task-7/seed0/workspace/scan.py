#!/usr/bin/env python3
"""Cluster above-table points from a camera's depth into object blobs.

Usage: python3 scan.py <camera> [zmin=0.435] [zmax=0.9]
Prints, per blob: pixel centroid, world centroid, world bbox, size.
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2


def grab(node, topic, msg_type, timeout=20.0):
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
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.435
    zmax = float(sys.argv[3]) if len(sys.argv) > 3 else 0.9
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
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    vs, us = np.mgrid[0:H, 0:W]
    z = depth
    ok = np.isfinite(z) & (z > 0)
    X = (us - cx) * z / fx
    Y = (vs - cy) * z / fy
    P = np.stack([X, Y, z], -1) @ R.T + T
    mask = ok & (P[..., 2] > zmin) & (P[..., 2] < zmax)
    m8 = mask.astype(np.uint8)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(m8, 8)
    print(f"camera {cam} at world {T.round(3)}; table z approx {np.median(P[ok][:,2]):.3f}")
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 15:
            continue
        sel = lab == i
        pts = P[sel]
        col = color[sel].mean(0)[::-1].astype(int)
        lo, hi = pts.min(0), pts.max(0)
        print(f"blob {i}: px({cents[i][0]:.0f},{cents[i][1]:.0f}) n={sel.sum()} "
              f"rgb={col} centre={pts.mean(0).round(3)} "
              f"xmin/max=({lo[0]:.3f},{hi[0]:.3f}) ymin/max=({lo[1]:.3f},{hi[1]:.3f}) "
              f"z=({lo[2]:.3f},{hi[2]:.3f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()


def profile(cam, cx_, cy_, r=0.06, zmin=0.43):
    """Print width-per-height of points near world (cx_, cy_)."""
    rclpy.init()
    node = rclpy.create_node("scan2")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    vs, us = np.mgrid[0:H, 0:W]
    ok = np.isfinite(depth) & (depth > 0)
    P = np.stack([(us - cx) * depth / fx, (vs - cy) * depth / fy, depth], -1) @ R.T + T
    sel = ok & (np.hypot(P[..., 0] - cx_, P[..., 1] - cy_) < r) & (P[..., 2] > zmin)
    pts = P[sel]
    for z0 in np.arange(zmin, pts[:, 2].max() + 0.01, 0.01):
        s = pts[(pts[:, 2] >= z0) & (pts[:, 2] < z0 + 0.01)]
        if len(s):
            print(f"z {z0:.2f}-{z0+0.01:.2f}: n={len(s):4d} x({s[:,0].min():.3f},{s[:,0].max():.3f}) "
                  f"y({s[:,1].min():.3f},{s[:,1].max():.3f}) cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
    rclpy.shutdown()


def blob_pca(cam, cx_, cy_, r=0.10, zmin=0.435, zmax=0.75):
    """Principal axis of the above-table points near world (cx_, cy_)."""
    rclpy.init()
    node = rclpy.create_node("scan3")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    vs, us = np.mgrid[0:H, 0:W]
    ok = np.isfinite(depth) & (depth > 0)
    P = np.stack([(us - cx) * depth / fx, (vs - cy) * depth / fy, depth], -1) @ R.T + T
    sel = ok & (np.hypot(P[..., 0] - cx_, P[..., 1] - cy_) < r) & (P[..., 2] > zmin) & (P[..., 2] < zmax)
    pts = P[sel]
    c = pts.mean(0)
    xy = pts[:, :2] - c[:2]
    w, v = np.linalg.eigh(xy.T @ xy / len(xy))
    axis = v[:, 1]
    along = xy @ axis
    across = xy @ v[:, 0]
    print(f"n={len(pts)} centre={c.round(4)} ztop={pts[:,2].max():.3f}")
    print(f"axis={axis.round(3)} yaw_deg={np.degrees(np.arctan2(axis[1], axis[0])):.1f} "
          f"length={along.max()-along.min():.3f} width={across.max()-across.min():.3f}")
    # centre of the widest part (body): use points in the lower 60% of along-range? print histogram
    for lo in np.arange(along.min(), along.max(), 0.01):
        s = across[(along >= lo) & (along < lo + 0.01)]
        if len(s):
            print(f"  along {lo:+.3f}: n={len(s):3d} width={s.max()-s.min():.3f}")
    rclpy.shutdown()
