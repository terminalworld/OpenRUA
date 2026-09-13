#!/usr/bin/env python3
"""Batch pixel->world for one camera. Usage: px.py <cam> u,v [u,v ...]
Also prints the camera pose and the world->panda_link0 transform."""
import struct, sys
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=20.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def q2R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tfmat(t):
    T = np.eye(4); T[:3, :3] = q2R(t.transform.rotation)
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def main():
    cam = sys.argv[1]
    pix = [tuple(int(x) for x in a.split(",")) for a in sys.argv[2:]]
    rclpy.init(); node = rclpy.create_node("px")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    import time
    end = time.time() + 10
    frame = f"{cam}_optical_frame"
    while time.time() < end and not (buf.can_transform("world", frame, rclpy.time.Time()) and buf.can_transform("world", "panda_link0", rclpy.time.Time())):
        rclpy.spin_once(node, timeout_sec=0.2)
    Tc = tfmat(buf.lookup_transform("world", frame, rclpy.time.Time()))
    Tb = tfmat(buf.lookup_transform("world", "panda_link0", rclpy.time.Time()))
    print("cam pos", Tc[:3, 3].round(3)); print("base pos", Tb[:3, 3].round(3))
    print("base R\n", Tb[:3, :3].round(3))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    d = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    for (u, v) in pix:
        z = float(d[v, u])
        p = Tc @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        pb = np.linalg.inv(Tb) @ p
        print(f"({u},{v}) depth={z:.3f} world={p[:3].round(4)} base={pb[:3].round(4)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
