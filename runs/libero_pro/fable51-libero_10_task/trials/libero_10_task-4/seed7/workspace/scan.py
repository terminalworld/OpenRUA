#!/usr/bin/env python3
"""Point-cloud the birdview depth into world frame and cluster objects above the table."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2

cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"

def grab(node, topic, T):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

rclpy.init(); node = rclpy.create_node("scan")
tfbuf = Buffer(); TransformListener(tfbuf, node)
depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
color = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
while not tfbuf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
Z = depth
X = (uu - cx) * Z / fx; Y = (vv - cy) * Z / fy
P = np.stack([X, Y, Z], -1) @ R.T + tr  # world coords per pixel
np.save(f"{cam}_world.npy", P)
zw = P[..., 2]
valid = np.isfinite(zw)
# table height: mode of z in the central region
tab = np.median(zw[valid & (np.abs(P[...,0]) < 0.3) & (np.abs(P[...,1]) < 0.5)])
print("table z ~", round(tab, 4))
mask = (valid & (zw > tab + 0.02) & (zw < tab + 0.4)).astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30: continue
    m = lab == i
    pts = P[m]
    bgr = color[m].mean(0)
    print(f"blob {i}: px_centroid=({cents[i][0]:.0f},{cents[i][1]:.0f}) area={stats[i,4]} "
          f"world x=[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y=[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
          f"ztop={pts[:,2].max():.3f} mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) color(bgr)={bgr.round()}")
rclpy.shutdown()
