#!/usr/bin/env python3
"""Segment objects above the table in the birdview depth and print world bboxes."""
import numpy as np, cv2, rclpy, yaml
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge

rclpy.init(); node = rclpy.create_node("ab")
got = {}
node.create_subscription(Image, "/birdview/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, "/birdview/color/camera_info", lambda m: got.setdefault("i", m), 1)
node.create_subscription(Image, "/birdview/color/image_raw", lambda m: got.setdefault("c", m), 1)
while len(got) < 3: rclpy.spin_once(node, timeout_sec=0.2)
d = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float32)
c = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
# camera at (-0.2, 0, 3.0), looking straight down; optical: x_img -> world?, check via quaternion
# q = (0.7071, 0.7071, 0, 0): R = rot 180deg about (1,1,0)/sqrt2 axis -> maps cam x->world y, cam y->world x, cam z->world -z
H, W = d.shape
vv, uu = np.mgrid[0:H, 0:W]
X = (uu - cx) * d / fx; Y = (vv - cy) * d / fy
wx = -0.2 + Y; wy = 0.0 + X; wz = 3.0 - d
np.save("bird_wz.npy", wz)
table = np.abs(wz - 0.90) < 0.005
above = (wz > 0.905) & (wz < 1.3) & (wx > -0.5) & (wx < 0.5) & (np.abs(wy) < 0.6)
n, lab, stats, cent = cv2.connectedComponentsWithStats(above.astype(np.uint8), 8)
for i in range(1, n):
    m = lab == i
    if m.sum() < 30: continue
    print(f"comp {i}: px={m.sum()} u={stats[i][0]}..{stats[i][0]+stats[i][2]} v={stats[i][1]}..{stats[i][1]+stats[i][3]}"
          f" wx={wx[m].min():.3f}..{wx[m].max():.3f} wy={wy[m].min():.3f}..{wy[m].max():.3f} zmax={wz[m].max():.3f} zmed={np.median(wz[m]):.3f}")
