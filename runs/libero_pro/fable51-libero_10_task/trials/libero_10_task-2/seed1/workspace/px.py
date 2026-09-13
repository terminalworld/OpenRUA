#!/usr/bin/env python3
"""Batch pixel->world for one camera: python3 px.py <cam> u,v [u,v ...]
Also saves <cam>_depth.npy. Uses the TF world->optical frame."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
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
    pts = [tuple(int(v) for v in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("px")
    buf = Buffer(); TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
    np.save(f"snaps/{cam}_depth.npy", depth)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    o = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    print(f"cam {cam} {depth.shape} fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")
    for (u, v) in pts:
        z = depth[v, u]
        p = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
        w = R @ p + o
        print(f"px({u},{v}) depth={z:.3f} -> world {w[0]:.4f} {w[1]:.4f} {w[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
