#!/usr/bin/env python3
"""Eye-in-hand depth -> world point cloud using FK for the camera pose.
Usage: python3 eih_cloud.py [out_prefix]
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as Rot
from rb import Robot

CAM = "robot0_eye_in_hand"
# camera optical frame relative to panda_hand (from tf_static)
T_HC = np.eye(4)
T_HC[:3, 3] = [0.050, 0.0, -0.001]
T_HC[:3, :3] = Rot.from_quat([0, 0, 0.707, 0.707]).as_matrix()


def grab(r, topic, msg_type):
    got = []
    sub = r.node.create_subscription(msg_type, topic, lambda m: got.append(m), 1)
    while not got:
        rclpy.spin_once(r.node, timeout_sec=0.3)
    r.node.destroy_subscription(sub)
    return got[0]


def cloud(r, prefix="eih"):
    import cv2
    p, R = r.fk()
    T_WH = np.eye(4); T_WH[:3, :3] = R; T_WH[:3, 3] = p
    T_WC = T_WH @ T_HC
    depth_msg = grab(r, f"/{CAM}/depth/image_raw", Image)
    info = grab(r, f"/{CAM}/color/camera_info", CameraInfo)
    color_msg = grab(r, f"/{CAM}/color/image_raw", Image)
    cv2.imwrite(f"{prefix}.png", CvBridge().imgmsg_to_cv2(color_msg, desired_encoding="bgr8"))
    depth = CvBridge().imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype(np.float64)
    H, W = depth.shape
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    P = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
    Pw = P @ T_WC[:3, :3].T + T_WC[:3, 3]
    Pw[~np.isfinite(depth) | (depth <= 0)] = np.nan
    np.save(f"{prefix}_xyz.npy", Pw)
    print(f"{prefix}_xyz.npy cam at", T_WC[:3, 3].round(3))
    return Pw


if __name__ == "__main__":
    r = Robot("eihcloud")
    cloud(r, sys.argv[1] if len(sys.argv) > 1 else "eih")
