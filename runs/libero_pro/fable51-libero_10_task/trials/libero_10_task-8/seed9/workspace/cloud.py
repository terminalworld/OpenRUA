#!/usr/bin/env python3
"""Depth image -> world-frame point cloud for one camera (fresh frame).

Usage as a module:  pts, rgb = cloud.grab("agentview")  # (N,3) world xyz
"""
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def _quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def grab(cam, node=None, timeout=20.0):
    own = node is None
    if own:
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node("cloud_" + cam)
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    got = {}
    subs = [
        node.create_subscription(Image, f"/{cam}/depth/image_raw",
                                 lambda m: got.setdefault("d", m), 1),
        node.create_subscription(Image, f"/{cam}/color/image_raw",
                                 lambda m: got.setdefault("c", m), 1),
        node.create_subscription(CameraInfo, f"/{cam}/color/camera_info",
                                 lambda m: got.setdefault("i", m), 1),
    ]
    frame = f"{cam}_optical_frame"
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(got) == 3 and tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    for s in subs:
        node.destroy_subscription(s)
    if len(got) < 3:
        raise RuntimeError(f"missing data for {cam}: {list(got)}")
    br = CvBridge()
    depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
    rgb = br.imgmsg_to_cv2(got["c"], "rgb8")
    k = got["i"].k
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = _quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y,
                  t.transform.translation.z])
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    z = depth
    X = (u - cx) * z / fx
    Y = (v - cy) * z / fy
    P = np.stack([X, Y, z], -1).reshape(-1, 3) @ R.T + T
    P = P.reshape(h, w, 3)
    if own:
        node.destroy_node()
    return P, rgb, depth


if __name__ == "__main__":
    import sys
    P, rgb, depth = grab(sys.argv[1])
    np.save(f"{sys.argv[1]}_cloud.npy", P)
    print(P.shape, "saved")
