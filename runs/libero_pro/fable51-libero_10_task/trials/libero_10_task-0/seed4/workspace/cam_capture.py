#!/usr/bin/env python3
"""Capture color, depth, intrinsics and world<-optical TF for a camera.

Usage: python3 cam_capture.py <camera>
Writes <camera>.png, <camera>_depth.npy, <camera>_meta.npz (K, T_world_cam).
"""
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("cam_capture")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    got = {}
    node.create_subscription(Image, f"/{cam}/color/image_raw",
                             lambda m: got.setdefault("color", m), 1)
    node.create_subscription(Image, f"/{cam}/depth/image_raw",
                             lambda m: got.setdefault("depth", m), 1)
    node.create_subscription(CameraInfo, f"/{cam}/color/camera_info",
                             lambda m: got.setdefault("info", m), 1)
    frame = f"{cam}_optical_frame"
    t0 = time.time()
    while time.time() - t0 < 60:
        rclpy.spin_once(node, timeout_sec=0.2)
        if all(k in got for k in ("color", "depth", "info")) and \
                tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    else:
        raise SystemExit(f"timeout; have {list(got)}")
    br = CvBridge()
    color = br.imgmsg_to_cv2(got["color"], "bgr8")
    depth = br.imgmsg_to_cv2(got["depth"], "passthrough").astype(np.float32)
    K = np.array(got["info"].k).reshape(3, 3)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z]
    cv2.imwrite(f"{cam}.png", color)
    np.save(f"{cam}_depth.npy", depth)
    np.savez(f"{cam}_meta.npz", K=K, T=T)
    print(f"saved {cam}: depth {depth.shape} range "
          f"{np.nanmin(depth):.3f}..{np.nanmax(depth):.3f}; cam at {T[:3,3]}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
