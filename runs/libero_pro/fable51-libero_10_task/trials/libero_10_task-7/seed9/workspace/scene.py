#!/usr/bin/env python3
"""Dump a world-frame point cloud from a camera and cluster objects above the table.
Usage: python3 scene.py <camera> [zmin_above_table]
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, typ):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(color_msg, "bgr8")
    frame = f"{cam}_optical_frame"
    while not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    z = depth
    pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=-1)
    world = pc @ R.T + T
    np.save(f"{cam}_world.npy", world)
    # table plane estimate: mode of z among points in the central region
    zs = world[..., 2]
    valid = np.isfinite(zs) & (z > 0.05)
    hist, edges = np.histogram(zs[valid], bins=400, range=(0, 2))
    table_z = float(sys.argv[3]) if len(sys.argv) > 3 else edges[np.argmax(hist)] + (edges[1] - edges[0]) / 2
    print(f"table_z ~ {table_z:.4f}")
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.01
    mask = valid & (zs > table_z + zmin) & (zs < table_z + 0.4)
    # exclude the robot: base frame at x=-0.51 world; robot occupies x < -0.3 roughly
    mask &= world[..., 0] > -0.35
    mask8 = mask.astype(np.uint8)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask8, 8)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 40:
            continue
        pts = world[labels == i]
        lo, hi = pts.min(0), pts.max(0)
        col = color[labels == i].mean(0)
        print(f"obj{i}: px_centroid=({cents[i][0]:.0f},{cents[i][1]:.0f}) area={stats[i, cv2.CC_STAT_AREA]} "
              f"x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] z[{lo[2]:.3f},{hi[2]:.3f}] "
              f"center=({(lo[0]+hi[0])/2:.3f},{(lo[1]+hi[1])/2:.3f}) bgr={col.astype(int)}")
    out = color.copy()
    out[mask] = (0.5 * out[mask] + [0, 127, 0]).astype(np.uint8)
    cv2.imwrite(f"{cam}_seg.png", out)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
