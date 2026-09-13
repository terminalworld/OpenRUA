#!/usr/bin/env python3
"""Dump a world-frame point cloud from a camera's depth + color and report
blobs standing above the table. Usage: python3 scene.py <camera>"""
import sys
import numpy as np
import rclpy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
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
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    while not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    v, u = np.mgrid[0:H, 0:W]
    z = depth
    pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], -1)
    pw = pc @ R.T + p0
    np.save(f"{cam}_pw.npy", pw)
    np.save(f"{cam}_color.npy", color)
    print("depth range", np.nanmin(z), np.nanmax(z))
    # table height estimate: mode of z in the central region
    zs = pw[..., 2]
    finite = zs[np.isfinite(zs)]
    hist, edges = np.histogram(finite, bins=200)
    table_z = edges[np.argmax(hist)]
    print("dominant z (table?)", table_z)
    mask = (zs > table_z + 0.015) & (zs < table_z + 0.5) & np.isfinite(zs)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 15:
            continue
        m = lab == i
        pts = pw[m]
        col = color[m].mean(0)
        x0, y0, w, h = stats[i, :4]
        print(f"blob {i}: px({cents[i][0]:.0f},{cents[i][1]:.0f}) area={stats[i, 4]} "
              f"bbox=({x0},{y0},{w},{h}) world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] "
              f"y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} "
              f"center=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) bgr={col.astype(int)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
