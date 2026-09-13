#!/usr/bin/env python3
"""Segment objects above the table from a camera's depth image -> world coords.
Usage: python3 scene.py <camera> [min_height_above_table=0.01]
"""
import sys
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2

cam = sys.argv[1]
hmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.01

rclpy.init()
node = rclpy.create_node("scene")
buf = Buffer(); TransformListener(buf, node)
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
frame = f"{cam}_optical_frame"
while not ({"d", "c", "i"} <= set(got) and buf.can_transform("world", frame, Time())):
    rclpy.spin_once(node, timeout_sec=0.2)
depth = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
t = buf.lookup_transform("world", frame, Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
Z = depth
X = (uu - cx) * Z / fx; Y = (vv - cy) * Z / fy
P = np.stack([X, Y, Z], -1) @ R.T + T   # HxWx3 world
valid = np.isfinite(Z) & (Z > 0.05)
zs = P[..., 2][valid]
# table height = most common z in the middle band
hist, edges = np.histogram(zs, bins=400)
table_z = edges[np.argmax(hist)] + (edges[1]-edges[0])/2
print(f"table_z ~ {table_z:.4f}")
np.save(f"{cam}_world.npy", P)
above = valid & (P[..., 2] > table_z + hmin) & (P[..., 2] < table_z + 0.5)
mask = above.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30: continue
    m = lab == i
    pts = P[m]
    lo, hi = pts.min(0), pts.max(0)
    cu, cv_ = cents[i]
    print(f"blob {i}: px=({cu:.0f},{cv_:.0f}) area={stats[i,4]} "
          f"x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] z[{lo[2]:.3f},{hi[2]:.3f}] "
          f"center=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) top={hi[2]:.3f}")
    x0, y0, w_, h_ = stats[i, :4]
    cv2.rectangle(color, (x0, y0), (x0+w_, y0+h_), (0, 255, 0), 1)
    cv2.putText(color, str(i), (x0, y0-2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
cv2.imwrite(f"{cam}_blobs.png", color)
rclpy.shutdown()
