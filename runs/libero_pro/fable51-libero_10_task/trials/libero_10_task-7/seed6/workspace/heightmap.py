#!/usr/bin/env python3
"""World-frame height map from one camera's depth; list blobs above the table."""
import sys, time
import numpy as np
import rclpy
import cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from px_batch import grab, quat_R

cam = sys.argv[1]
zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.435
zmax = float(sys.argv[3]) if len(sys.argv) > 3 else 0.7
rclpy.init()
node = rclpy.create_node("hm")
buf = Buffer(); TransformListener(buf, node)
depth = grab(node, f"/{cam}/depth/image_raw", Image)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
color = grab(node, f"/{cam}/color/image_raw", Image)
C = np.frombuffer(color.data, dtype=np.uint8).reshape(color.height, color.width, -1)
D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
frame = f"{cam}_optical_frame"
end = time.time() + 20
while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation
R = quat_R(q.x, q.y, q.z, q.w)
o = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
vs, us = np.mgrid[0:depth.height, 0:depth.width]
P = np.stack([(us - cx) * D / fx, (vs - cy) * D / fy, D], -1) @ R.T + o
np.save(f"snaps/{cam}_world.npy", P)
Z = P[..., 2]
mask = ((Z > zmin) & (Z < zmax) & np.isfinite(Z)).astype(np.uint8)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 15:
        continue
    m = lab == i
    pts = P[m]
    col = C[m].mean(0)
    print(f"blob {i}: px centroid ({cent[i][0]:.0f},{cent[i][1]:.0f}) area {stats[i,4]} "
          f"world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
          f"z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean xyz ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f},{pts[:,2].mean():.3f}) color {col.astype(int)}")
cv2.imwrite(f"snaps/{cam}_mask.png", mask * 255)
rclpy.shutdown()
