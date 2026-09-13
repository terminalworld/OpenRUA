#!/usr/bin/env python3
"""Build a world-frame point cloud from a camera and segment objects above the table.

Usage: python3 scene.py <camera>
Saves <camera>_cloud.npz (points Nx3 world, pixel indices) and prints
world→panda_link0 and clusters of above-table points.
"""
import struct
import sys

import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def tf_mat(tfbuf, node, target, source):
    import time
    end = time.time() + 15
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform(target, source, Time()):
            break
    t = tfbuf.lookup_transform(target, source, Time())
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    H, W = depth_msg.height, depth_msg.width
    depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(H, W)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    T = tf_mat(tfbuf, node, "world", f"{cam}_optical_frame")
    try:
        Tb = tf_mat(tfbuf, node, "world", "panda_link0")
        print("world<-panda_link0:\n", np.round(Tb, 4))
    except Exception as e:
        print("no world<-panda_link0 tf:", e)
    vs, us = np.mgrid[0:H, 0:W]
    z = depth
    ok = np.isfinite(z) & (z > 0)
    X = (us - cx) * z / fx
    Y = (vs - cy) * z / fy
    P = np.stack([X, Y, z, np.ones_like(z)], -1) @ T.T
    P = P[..., :3]
    np.savez(f"{cam}_cloud.npz", P=P, ok=ok)
    print("camera pose world:", np.round(T[:3, 3], 3))
    pts = P[ok]
    # table height estimate: most common z
    hist, edges = np.histogram(pts[:, 2], bins=400)
    table_z = edges[np.argmax(hist)]
    print(f"table z ~ {table_z:.3f}")
    above = ok & (P[..., 2] > table_z + 0.015)
    # connected components in the image on the above mask
    import cv2
    n, lab = cv2.connectedComponents(above.astype(np.uint8))
    for i in range(1, n):
        m = lab == i
        if m.sum() < 30:
            continue
        p = P[m]
        vv, uu = np.where(m)
        print(f"cluster {i}: n={m.sum()} px u[{uu.min()}-{uu.max()}] v[{vv.min()}-{vv.max()}] "
              f"x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] "
              f"z[{p[:,2].min():.3f},{p[:,2].max():.3f}] centroid={np.round(p.mean(0),3)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
