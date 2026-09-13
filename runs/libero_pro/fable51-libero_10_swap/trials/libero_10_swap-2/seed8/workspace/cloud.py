#!/usr/bin/env python3
"""Build a world-frame point cloud from a camera's current depth frame.

Usage: python3 cloud.py <camera> [out.npz]
Saves xyz (H,W,3) world coords + color (H,W,3) to <camera>_cloud.npz
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, typ):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else f"{cam}_cloud.npz"
    rclpy.init(); node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "rgb8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    while not buf.can_transform("world", frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, Time())
    q, tr = t.transform.rotation, t.transform.translation
    R = quat_R(q.x, q.y, q.z, q.w); T = np.array([tr.x, tr.y, tr.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
    xyz = pc @ R.T + T
    np.savez(out, xyz=xyz, color=color, depth=depth)
    print(out, xyz.shape)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
