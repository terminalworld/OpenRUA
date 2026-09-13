#!/usr/bin/env python3
"""Segment tabletop objects from a top-down camera: cluster world points
with z in a height band, print each cluster's centroid, extent, top z,
and mean color. Usage: python3 segment.py [cam=birdview] [zmin] [zmax]
"""
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"
zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.50
zmax = float(sys.argv[3]) if len(sys.argv) > 3 else 0.66


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


rclpy.init()
node = rclpy.create_node("segment")
buf = Buffer()
TransformListener(buf, node)
depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough")
color = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "rgb8")
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
while not buf.can_transform("world", frame, Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, Time())
q, tr = t.transform.rotation, t.transform.translation
R = quat_R(q.x, q.y, q.z, q.w)
T = np.array([tr.x, tr.y, tr.z])

fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
z = depth.astype(np.float64)
pc = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z], -1)
pw = pc @ R.T + T
mask = np.isfinite(z) & (pw[..., 2] > zmin) & (pw[..., 2] < zmax)

# connected components on the mask
import cv2
n, lab = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
for i in range(1, n):
    m = lab == i
    if m.sum() < 30:
        continue
    P = pw[m]
    C = color[m].reshape(-1, 3).mean(0)
    us, vs = uu[m], vv[m]
    print(f"cluster {i}: n={m.sum()} centroid=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) "
          f"x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] "
          f"ztop={P[:,2].max():.3f} px=({us.mean():.0f},{vs.mean():.0f}) rgb={C.astype(int)}")
rclpy.shutdown()
