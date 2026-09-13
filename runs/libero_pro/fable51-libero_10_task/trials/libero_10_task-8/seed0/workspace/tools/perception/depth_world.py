#!/usr/bin/env python3
"""Grab depth+intrinsics+TF for a camera; save world-XYZ point cloud as npy.
Usage: depth_world.py <camera> -> img/<camera>_xyz.npy (H,W,3) world coords
"""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

def grab(node, topic, T, timeout=30):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time; end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("depth_world")
buf = Buffer(); TransformListener(buf, node)
depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
import time; end = time.time() + 20
while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = depth.shape
u, v = np.meshgrid(np.arange(W), np.arange(H))
pc = np.stack([(u-cx)*depth/fx, (v-cy)*depth/fy, depth], -1)
xyz = pc @ R.T + tr
np.save(f"img/{cam}_xyz.npy", xyz)
print("saved", xyz.shape, "depth range", np.nanmin(depth), np.nanmax(depth))
rclpy.shutdown()
