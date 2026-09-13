#!/usr/bin/env python3
"""Segment above-table blobs in a camera's depth frame and print world
bounding boxes. Usage: python3 objs.py <camera> [min_height=0.01]"""
import sys, time
import numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scene import grab, tf_to_T

cam = sys.argv[1]
minh = float(sys.argv[2]) if len(sys.argv) > 2 else 0.01
TABLE_Z = 0.4255
rclpy.init(); node = rclpy.create_node("objs")
tfbuf = Buffer(); TransformListener(tfbuf, node)
depth = grab(node, f"/{cam}/depth/image_raw", Image)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
end = time.time() + 10
while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
T = tf_to_T(tfbuf.lookup_transform("world", frame, rclpy.time.Time()))
D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
v, u = np.mgrid[0:depth.height, 0:depth.width]
P = np.stack([(u - cx) * D / fx, (v - cy) * D / fy, D, np.ones_like(D)], -1)
W = P @ T.T
Z = W[..., 2]
mask = (Z > TABLE_Z + minh) & np.isfinite(D) & (D > 0)
n, lab = cv2.connectedComponents(mask.astype(np.uint8))
for i in range(1, n):
    m = lab == i
    if m.sum() < 15: continue
    pts = W[m]
    vs, us = np.nonzero(m)
    print(f"blob{i}: px u[{us.min()}-{us.max()}] v[{vs.min()}-{vs.max()}] n={m.sum()} "
          f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
          f"z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
rclpy.shutdown()
