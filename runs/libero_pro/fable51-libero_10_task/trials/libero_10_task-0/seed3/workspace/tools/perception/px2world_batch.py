#!/usr/bin/env python3
"""Batch pixel->world: python3 px2world_batch.py <camera> u,v [u,v ...]
Also prints a depth patch median option: use u,v,r to take median depth over (2r+1)^2 window."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

def _grab(node, topic, msg_type, timeout=15.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]

def main():
    cam = sys.argv[1]
    rclpy.init(); node = rclpy.create_node("px2world_batch")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth = _grab(node, f"/{cam}/depth/image_raw", Image)
    info = _grab(node, f"/{cam}/color/camera_info", CameraInfo)
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()): break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation; x, y, zz, w = q.x, q.y, q.z, q.w
    R = np.array([[1-2*(y*y+zz*zz), 2*(x*y-zz*w), 2*(x*zz+y*w)],
                  [2*(x*y+zz*w), 1-2*(x*x+zz*zz), 2*(y*zz-x*w)],
                  [2*(x*zz-y*w), 2*(y*zz+x*w), 1-2*(x*x+y*y)]])
    T = np.eye(4); T[:3,:3] = R
    T[:3,3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    for a in sys.argv[2:]:
        parts = [int(v) for v in a.split(",")]
        u, v = parts[0], parts[1]; r = parts[2] if len(parts) > 2 else 0
        patch = D[max(0,v-r):v+r+1, max(0,u-r):u+r+1]
        patch = patch[np.isfinite(patch) & (patch > 0)]
        if patch.size == 0:
            print(f"{u},{v}: no depth"); continue
        z = float(np.median(patch))
        p = T @ np.array([(u-cx)*z/fx, (v-cy)*z/fy, z, 1.0])
        print(f"{u},{v}: depth={z:.3f} world=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
