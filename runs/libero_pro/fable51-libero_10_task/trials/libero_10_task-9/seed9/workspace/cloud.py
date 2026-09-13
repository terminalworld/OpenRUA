#!/usr/bin/env python3
"""Grab depth+color+intrinsics+TF for a camera, save a world-frame point cloud
as <cam>_cloud.npz (xyz world, rgb, pixel uv). Usage: cloud.py <cam>"""
import sys
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener


def grab(node, topic, mt):
    got = {}
    sub = node.create_subscription(mt, topic, lambda m: got.setdefault("m", m), 1)
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
    rclpy.init()
    node = rclpy.create_node("cloud")
    buf = Buffer()
    TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(color_msg, "rgb8")
    frame = f"{cam}_optical_frame"
    while not buf.can_transform("world", frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, Time())
    q, tr = t.transform.rotation, t.transform.translation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([tr.x, tr.y, tr.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    vs, us = np.mgrid[0:H, 0:W]
    z = depth
    X = (us - cx) * z / fx
    Y = (vs - cy) * z / fy
    pc = np.stack([X, Y, z], -1).reshape(-1, 3)
    pw = pc @ R.T + T
    valid = np.isfinite(z).reshape(-1) & (z.reshape(-1) > 0)
    np.savez(f"{cam}_cloud.npz", xyz=pw.reshape(H, W, 3), rgb=color, valid=valid.reshape(H, W))
    print(f"saved {cam}_cloud.npz  H={H} W={W} fx={fx:.1f} cx={cx:.1f} cy={cy:.1f}")
    print(f"depth range {np.nanmin(z):.3f}..{np.nanmax(z):.3f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
