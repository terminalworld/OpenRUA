#!/usr/bin/env python3
"""Dump a camera's depth + TF as a world-frame point cloud helper.

Usage: python3 scene3d.py <camera>
Saves <camera>_xyz.npy (H x W x 3 world coords, NaN where invalid) and
prints the camera pose in world.
"""
import sys
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge


def grab(node, topic, typ, qos=1):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), qos)
    for _ in range(300):
        if "m" in got:
            break
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
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
    node = rclpy.create_node("scene3d")
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    # collect TF (static + dynamic) for a moment
    tfs = {}
    def on_tf(m):
        for t in m.transforms:
            tfs[t.child_frame_id] = (t.header.frame_id, t.transform)
    qos_tl = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TFMessage, "/tf_static", on_tf, qos_tl)
    node.create_subscription(TFMessage, "/tf", on_tf, 100)
    frame = f"{cam}_optical_frame"
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.2)
        if frame in tfs:
            break
    if frame not in tfs:
        print("known frames:", sorted(tfs))
        raise SystemExit(f"no TF for {frame}")
    # chain up to world
    T = np.eye(4)
    f = frame
    while f != "world":
        parent, tr = tfs[f]
        Ti = np.eye(4)
        q = tr.rotation
        Ti[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
        Ti[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
        T = Ti @ T
        f = parent
    print("cam->world T:\n", np.round(T, 4))
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    H, W = depth.shape
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth, np.ones_like(depth)], -1) @ T.T
    xyz = P[..., :3]
    xyz[~np.isfinite(depth) | (depth <= 0)] = np.nan
    np.save(f"{cam}_xyz.npy", xyz)
    np.save(f"{cam}_depth.npy", depth)
    print("saved", f"{cam}_xyz.npy", "depth range", np.nanmin(depth), np.nanmax(depth))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
