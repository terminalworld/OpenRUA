#!/usr/bin/env python3
"""Dump a world-frame point cloud (H,W,3) of a camera as <cam>_xyz.npy plus color."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2

def grab(node, topic, T, timeout=30):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time; end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
buf = Buffer(); TransformListener(buf, node)
depth = grab(node, f"/{cam}/depth/image_raw", Image)
color = grab(node, f"/{cam}/color/image_raw", Image)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
d = CvBridge().imgmsg_to_cv2(depth, "passthrough").astype(np.float64)
bgr = CvBridge().imgmsg_to_cv2(color, "bgr8")
frame = f"{cam}_optical_frame"
import time; end = time.time() + 10
while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation; R = quat_R(q.x, q.y, q.z, q.w)
tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = d.shape
u, v = np.meshgrid(np.arange(W), np.arange(H))
pc = np.stack([(u-cx)*d/fx, (v-cy)*d/fy, d], -1)
xyz = pc @ R.T + tr
np.save(f"{cam}_xyz.npy", xyz); cv2.imwrite(f"{cam}.png", bgr)
print("saved", xyz.shape, "cam pos", tr)
rclpy.shutdown()
