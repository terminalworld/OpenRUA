#!/usr/bin/env python3
"""Project a whole camera depth frame into world coords and report
objects: usage scene3d.py <camera>  -> saves <camera>_world.npy (HxWx3)
and prints table-height stats."""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene3d")
    buf = Buffer(); TransformListener(buf, node)
    d = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
    frame = f"{cam}_optical_frame"
    while not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    v, u = np.mgrid[0:d.height, 0:d.width]
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + tr
    np.save(f"{cam}_world.npy", P)
    print("cam pos", tr, "depth range", np.nanmin(depth), np.nanmax(depth))
    z = P[..., 2]
    print("z percentiles", np.nanpercentile(z, [5, 25, 50, 75, 95]))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
