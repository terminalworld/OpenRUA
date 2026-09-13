#!/usr/bin/env python3
"""Grab depth+intrinsics+TF for a camera once, then print world coords for
many pixels. Usage: scene.py <camera> u,v [u,v ...]  (also dumps world->panda_link0)
"""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, DurabilityPolicy


def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    import time
    t0 = time.time()
    while "m" not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def tf_mat(t):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4); T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def main():
    cam = sys.argv[1]
    pix = [tuple(int(a) for a in p.split(",")) for p in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("scene")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    d = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    np.save(f"{cam}_depth.npy", d)
    frame = f"{cam}_optical_frame"
    import time
    t0 = time.time()
    while time.time() - t0 < 10 and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    T = tf_mat(buf.lookup_transform("world", frame, rclpy.time.Time()))
    if buf.can_transform("world", "panda_link0", rclpy.time.Time()):
        Tb = tf_mat(buf.lookup_transform("world", "panda_link0", rclpy.time.Time()))
        print("world->panda_link0:", np.round(Tb[:3, 3], 4), "R diag", np.round(np.diag(Tb[:3, :3]), 3))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    print("cam pos", np.round(T[:3, 3], 3))
    for (u, v) in pix:
        z = d[v, u]
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"({u},{v}) depth={z:.4f} world= {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
