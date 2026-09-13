#!/usr/bin/env python3
"""Grab depth+info+TF for a camera, save world-frame XYZ array (H,W,3) to <cam>_xyz.npy."""
import sys
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=30.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
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
    rclpy.init()
    node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not buf.can_transform("world", frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    p = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u = np.arange(W)[None, :].repeat(H, 0)
    v = np.arange(H)[:, None].repeat(W, 1)
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    pc = np.stack([X, Y, depth], -1)
    world = pc @ R.T + p
    np.save(f"img/{cam}_xyz.npy", world)
    np.save(f"img/{cam}_depth.npy", depth)
    print("cam pos", p, "R", R.round(3).tolist())
    print("depth range", np.nanmin(depth), np.nanmax(depth))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
