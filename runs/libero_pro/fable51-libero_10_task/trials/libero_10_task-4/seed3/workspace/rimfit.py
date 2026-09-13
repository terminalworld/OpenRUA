#!/usr/bin/env python3
"""Fit a circle (in world XY) to the rim points of each mug seen by a camera.
Usage: python3 rimfit.py <cam> [zmin] [zmax] — rim points = z within band.
Prints centre, radius, top z for each cluster."""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from rob import quat_R

cam = sys.argv[1] if len(sys.argv) > 1 else "robot0_eye_in_hand"
zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.56
zmax = float(sys.argv[3]) if len(sys.argv) > 3 else 0.63


def grab(node, topic, typ):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def fit_circle(x, y):
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]
    b = x * x + y * y
    c, *_ = np.linalg.lstsq(A, b, rcond=None)
    r = np.sqrt(c[2] + c[0] ** 2 + c[1] ** 2)
    return c[0], c[1], r


rclpy.init()
node = rclpy.create_node("rimfit")
buf = Buffer()
TransformListener(buf, node)
depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough")
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
pw = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z], -1) @ R.T + T
mask = np.isfinite(z) & (pw[..., 2] > zmin) & (pw[..., 2] < zmax)
n, lab = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
for i in range(1, n):
    m = lab == i
    if m.sum() < 200:
        continue
    P = pw[m]
    # use only the topmost 1.5 cm of the cluster (the rim ring itself)
    top = P[:, 2].max()
    ring = P[P[:, 2] > top - 0.015]
    cxw, cyw, rad = fit_circle(ring[:, 0], ring[:, 1])
    print(f"cluster {i}: n={m.sum()} ring_n={len(ring)} rim_center=({cxw:.4f},{cyw:.4f}) "
          f"radius={rad:.4f} ztop={top:.4f} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] "
          f"y[{P[:,1].min():.3f},{P[:,1].max():.3f}]")
rclpy.shutdown()
