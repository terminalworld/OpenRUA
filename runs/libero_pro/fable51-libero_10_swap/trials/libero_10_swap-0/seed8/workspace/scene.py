#!/usr/bin/env python3
"""Grab color+depth+info from a camera, print world coords for pixels.

Usage: python3 scene.py <camera> u,v [u,v ...]
Also prints the world->panda_link0 transform and saves <camera>_cloud.npy
(HxWx3 world xyz) for offline analysis.
"""
import struct
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def T_of(t):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z]
    return T


def main():
    cam = sys.argv[1]
    pix = [tuple(int(x) for x in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype(np.float64)
    import time
    end = time.time() + 15
    frame = f"{cam}_optical_frame"
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()) and \
           tfbuf.can_transform("world", "panda_link0", rclpy.time.Time()):
            break
    Tc = T_of(tfbuf.lookup_transform("world", frame, rclpy.time.Time()))
    Tb = T_of(tfbuf.lookup_transform("world", "panda_link0", rclpy.time.Time()))
    print("world->panda_link0 translation:", np.round(Tb[:3, 3], 4))
    print("world->panda_link0 rotation:\n", np.round(Tb[:3, :3], 4))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - cx) * depth / fx
    Y = (vs - cy) * depth / fy
    P = np.stack([X, Y, depth, np.ones_like(depth)], -1)
    Pw = P @ Tc.T
    cloud = Pw[..., :3]
    np.save(f"{cam}_cloud.npy", cloud)
    np.save(f"{cam}_depth.npy", depth)
    for (u, v) in pix:
        print(f"px ({u},{v}) depth={depth[v, u]:.4f} world=", np.round(cloud[v, u], 4))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
