#!/usr/bin/env python3
"""Batch pixel -> world for one camera. Usage: px_batch.py <cam> u,v [u,v ...]
Also saves <cam>_depth.npy and prints the camera pose."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=30.0):
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
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    cam = sys.argv[1]
    pix = [tuple(int(a) for a in p.split(",")) for p in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("px_batch")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    np.save(f"snaps/{cam}_depth.npy", D)
    import time
    end = time.time() + 20
    frame = f"{cam}_optical_frame"
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    o = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    print(f"cam {cam} origin {o} fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} size {depth.width}x{depth.height}")
    for (u, v) in pix:
        z = float(D[v, u])
        p = R @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z]) + o
        print(f"px ({u},{v}) depth {z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
