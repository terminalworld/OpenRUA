#!/usr/bin/env python3
"""Save a world-frame XYZ cloud (H,W,3) for a camera as <cam>_xyz.npy."""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from px_batch import grab, quat_T


def main():
    cam = sys.argv[1]
    rclpy.init(); node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buf.can_transform("world", frame, rclpy.time.Time()):
            break
    T = quat_T(buf.lookup_transform("world", frame, rclpy.time.Time()))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    v, u = np.mgrid[0:depth.height, 0:depth.width]
    P = np.stack([(u - cx) * D / fx, (v - cy) * D / fy, D, np.ones_like(D)], -1)
    W = P @ T.T
    np.save(f"{cam}_xyz.npy", W[..., :3].astype(np.float32))
    print("saved", f"{cam}_xyz.npy", W.shape)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
