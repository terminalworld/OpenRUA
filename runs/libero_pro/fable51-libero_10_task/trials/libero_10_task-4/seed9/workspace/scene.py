#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera, save arrays, and print
world coords for requested pixels.  Usage: scene.py <cam> [u,v ...]"""
import sys, struct
import numpy as np, rclpy, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


def grab(node, topic, typ, timeout=30.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
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
    rclpy.init(); node = rclpy.create_node("scene")
    buf = Buffer(); TransformListener(buf, node)
    br = CvBridge()
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end and not buf.can_transform("world", frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, Time()).transform
    T = np.eye(4); T[:3, :3] = quat_R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    K = np.array(info.k).reshape(3, 3)
    cv2.imwrite(f"{cam}.png", color)
    np.save(f"{cam}_depth.npy", depth); np.save(f"{cam}_T.npy", T); np.save(f"{cam}_K.npy", K)
    print("K", K.tolist()); print("T", T.round(4).tolist())
    for a in sys.argv[2:]:
        u, v = map(int, a.split(","))
        z = depth[v, u]
        p = T @ np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, 1.0])
        print(f"px({u},{v}) depth={z:.4f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
