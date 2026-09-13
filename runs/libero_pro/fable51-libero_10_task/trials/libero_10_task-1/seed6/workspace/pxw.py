#!/usr/bin/env python3
"""Batch pixel->world for one camera: python3 pxw.py <cam> u,v [u,v ...]
Also supports a window median: u,v,r -> median depth in (2r+1)^2 window.
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


def grab(node, topic, typ, timeout=15.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("pxw")
    buf = Buffer(); TransformListener(buf, node)
    depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    T = np.eye(4); T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    for arg in sys.argv[2:]:
        parts = [int(a) for a in arg.split(",")]
        u, v = parts[0], parts[1]
        r = parts[2] if len(parts) > 2 else 0
        win = depth[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
        win = win[np.isfinite(win) & (win > 0)]
        if win.size == 0:
            print(f"({u},{v}) no depth"); continue
        z = float(np.median(win))
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"({u},{v}) depth={z:.3f} world=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
