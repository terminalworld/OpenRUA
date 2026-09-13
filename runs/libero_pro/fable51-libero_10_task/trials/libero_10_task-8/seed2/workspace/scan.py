#!/usr/bin/env python3
"""Segment objects above the table in a top-down camera; print world stats."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"


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


rclpy.init()
node = rclpy.create_node("scan")
buf = Buffer(); TransformListener(buf, node)
d = grab(node, f"/{cam}/depth/image_raw", Image)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
frame = f"{cam}_optical_frame"
while not buf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation
R = quat_R(q.x, q.y, q.z, q.w)
tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
v, u = np.mgrid[0:d.height, 0:d.width]
z = depth
pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], -1)
W = pc @ R.T + tr
np.save(f"{cam}_world.npy", W)
print("camera at", tr, "R=\n", R)
# objects above table (z>0.905) within table region
mask = (W[..., 2] > 0.905) & np.isfinite(W[..., 2])
from scipy import ndimage
lab, n = ndimage.label(mask)
for i in range(1, n + 1):
    m = lab == i
    if m.sum() < 30:
        continue
    P = W[m]
    print(f"blob {i}: n={m.sum()} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] "
          f"y[{P[:,1].min():.3f},{P[:,1].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}] "
          f"centroid=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) "
          f"px u[{u[m].min()},{u[m].max()}] v[{v[m].min()},{v[m].max()}]")
rclpy.shutdown()
