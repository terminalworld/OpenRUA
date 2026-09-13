#!/usr/bin/env python3
"""Segment objects above the table from a top-down camera; print world xyz clusters.
Usage: python3 locate.py <camera> [min_height_above_table]
"""
import sys, struct
import numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from cv_bridge import CvBridge

cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"
minh = float(sys.argv[2]) if len(sys.argv) > 2 else 0.015

rclpy.init(); node = rclpy.create_node("locate")
buf = Buffer(); TransformListener(buf, node)
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
while not all(k in got for k in "dci") or not buf.can_transform("world", f"{cam}_optical_frame", Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
depth = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
K = np.array(got["i"].k).reshape(3, 3)
t = buf.lookup_transform("world", f"{cam}_optical_frame", Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
Z = depth
X = (uu - K[0, 2]) * Z / K[0, 0]; Y = (vv - K[1, 2]) * Z / K[1, 1]
P = np.stack([X, Y, Z], -1) @ R.T + T
valid = np.isfinite(Z) & (Z > 0.05)
# table height = mode of z among valid points in the central region
zs = P[..., 2][valid]
hist, edges = np.histogram(zs, bins=400, range=(0, 2))
table_z = float(sys.argv[3]) if len(sys.argv) > 3 else edges[np.argmax(hist)] + (edges[1]-edges[0])/2
print(f"table_z ~ {table_z:.4f}")
mask = valid & (P[..., 2] > table_z + minh) & (P[..., 2] < table_z + 0.5)
mask = mask.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
out = color.copy()
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 15: continue
    m = lab == i
    pts = P[m]
    col = color[m].mean(0)
    cx, cy = cents[i]
    print(f"cluster {i}: px=({cx:.0f},{cy:.0f}) area={stats[i,4]} "
          f"world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
          f"ztop={pts[:,2].max():.3f} zmean={pts[:,2].mean():.3f} centre=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) bgr={col.astype(int)}")
    cv2.rectangle(out, (stats[i,0], stats[i,1]), (stats[i,0]+stats[i,2], stats[i,1]+stats[i,3]), (0,255,0), 1)
    cv2.putText(out, str(i), (int(cx), int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,255), 1)
cv2.imwrite(f"{cam}_seg.png", out)
rclpy.shutdown()
