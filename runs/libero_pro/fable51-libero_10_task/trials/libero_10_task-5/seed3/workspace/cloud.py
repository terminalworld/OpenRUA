#!/usr/bin/env python3
"""Grab depth+intrinsics+TF for a camera, save world-frame point cloud as npz.
Usage: python3 cloud.py <camera>   -> <camera>_cloud.npz (xyz: HxWx3, rgb: HxWx3)
"""
import sys, struct
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


def grab(node, topic, T, timeout=30.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time; end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    cam = sys.argv[1]
    rclpy.init(); node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    rgb = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "rgb8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time; end = time.time() + 15
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation; R = quat_R(q.x, q.y, q.z, q.w)
    p = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
    xyz = pc @ R.T + p
    np.savez(f"{cam}_cloud.npz", xyz=xyz, rgb=rgb, depth=depth, K=np.array(info.k).reshape(3, 3), R=R, p=p)
    print(cam, "cam pos", p, "depth range", np.nanmin(depth), np.nanmax(depth))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
