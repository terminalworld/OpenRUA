#!/usr/bin/env python3
"""Grab birdview color+depth, back-project to world, and report object blobs.

Usage: python3 locate.py [cam=birdview]
Saves <cam>_pts.npz with world xyz per pixel and prints table-height stats plus
blob centroids for colour masks.
"""
import sys
import numpy as np
import rclpy, cv2
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"
rclpy.init()
node = rclpy.create_node("locate")
buf = Buffer(); TransformListener(buf, node)
got = {}
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
while not all(k in got for k in "cdi") or not buf.can_transform("world", f"{cam}_optical_frame", Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
br = CvBridge()
color = br.imgmsg_to_cv2(got["c"], "bgr8")
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
t = buf.lookup_transform("world", f"{cam}_optical_frame", Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
pc = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1)
pw = pc @ R.T + T
np.savez(f"{cam}_pts.npz", pw=pw, color=color, depth=depth)
cv2.imwrite(f"{cam}.png", color)
valid = np.isfinite(depth) & (depth > 0)
print("world z percentiles:", np.percentile(pw[valid][:, 2], [1, 5, 50, 95, 99]))
# table height estimate: most common z
hist, edges = np.histogram(pw[valid][:, 2], bins=200)
zt = edges[np.argmax(hist)]
print("table z ~", zt)
above = valid & (pw[:, :, 2] > zt + 0.01)
n, lab, stats, cents = cv2.connectedComponentsWithStats(above.astype(np.uint8), 8)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30:
        continue
    m = lab == i
    p = pw[m]
    c = color[m].mean(0)
    print(f"blob {i}: area={stats[i,4]} px bbox(u,v,w,h)={stats[i,:4].tolist()} "
          f"centroid_px=({cents[i][0]:.0f},{cents[i][1]:.0f}) world x[{p[:,0].min():.3f},{p[:,0].max():.3f}] "
          f"y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) BGR={c.round(0)}")
rclpy.shutdown()
