#!/usr/bin/env python3
"""Eye-in-hand 3D check: point cloud in the camera frame; report the
fingertip blocks (bottom of image) and the nearest rim points."""
import numpy as np, rclpy, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

CAM = "robot0_eye_in_hand"


def grab(node, topic, T):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


rclpy.init(); node = rclpy.create_node("eih_check"); br = CvBridge()
color = br.imgmsg_to_cv2(grab(node, f"/{CAM}/color/image_raw", Image), "bgr8")
depth = br.imgmsg_to_cv2(grab(node, f"/{CAM}/depth/image_raw", Image), "passthrough").astype(float)
info = grab(node, f"/{CAM}/color/camera_info", CameraInfo)
rclpy.shutdown()
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = depth.shape
vv, uu = np.mgrid[0:H, 0:W]
X = (uu - cx) * depth / fx; Y = (vv - cy) * depth / fy; Z = depth
np.save("eih_depth.npy", depth); cv2.imwrite("eih_color.png", color)
print(f"size {W}x{H} fx={fx:.1f} depth range {np.nanmin(Z):.3f}..{np.nanmax(Z):.3f}")
# depth histogram of near things (fingers)
near = Z < 0.20
print("near (<0.20m) pixel count", near.sum())
for lo in np.arange(0.05, 0.40, 0.025):
    s = (Z >= lo) & (Z < lo + 0.025)
    if s.sum() > 50:
        print(f"  Z {lo:.3f}-{lo+0.025:.3f}: n={s.sum():6d} X[{X[s].min():+.3f},{X[s].max():+.3f}] Y[{Y[s].min():+.3f},{Y[s].max():+.3f}]")
