#!/usr/bin/env python3
"""Dump a camera's depth frame as a world-frame point cloud (.npz) plus
a height-above-table mask image. Usage: python3 cloud.py <camera>"""
import struct, sys
import numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


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
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
buf = Buffer(); TransformListener(buf, node)
d = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
while not buf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation
R = quat_R(q.x, q.y, q.z, q.w)
p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
h, w = d.shape
u, v = np.meshgrid(np.arange(w), np.arange(h))
pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], -1)
P = pc @ R.T + p0
np.savez(f"{cam}_cloud.npz", P=P, depth=d)
print("cam pos", p0, "R", R.round(3).tolist())
z = P[..., 2]
print("z percentiles", np.nanpercentile(z[np.isfinite(z)], [1, 5, 25, 50, 75, 95, 99]).round(3))
rclpy.shutdown()
