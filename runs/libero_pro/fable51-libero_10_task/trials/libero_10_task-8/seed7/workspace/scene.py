#!/usr/bin/env python3
"""Dump a world-frame height map from a camera; locate objects above the table."""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, typ, timeout=30.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
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
    node = rclpy.create_node("scene")
    buf = Buffer(); TransformListener(buf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    while not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    v, u = np.mgrid[0:H, 0:W]
    pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
    pw = pc @ R.T + T
    np.save(f"{cam}_pw.npy", pw)
    cv2.imwrite(f"{cam}_color.png", color)
    z = pw[..., 2]
    print("depth range", np.nanmin(depth), np.nanmax(depth))
    print("z percentiles", np.nanpercentile(z, [1, 5, 50, 95, 99]))
    # table height = mode of z
    hist, edges = np.histogram(z[np.isfinite(z)], bins=200)
    table = edges[np.argmax(hist)]
    print("table z ~", table)
    mask = (z > table + 0.02) & np.isfinite(z)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 30:
            continue
        m = lab == i
        pts = pw[m]
        print(f"blob {i}: px centroid ({cents[i][0]:.0f},{cents[i][1]:.0f}) area {stats[i,4]} "
              f"world xy min {pts[:,0].min():.3f},{pts[:,1].min():.3f} max {pts[:,0].max():.3f},{pts[:,1].max():.3f} "
              f"mean {pts[:,0].mean():.3f},{pts[:,1].mean():.3f} zmax {pts[:,2].max():.3f} "
              f"color {color[m].mean(0).astype(int)}")
    rclpy.shutdown()


main()
