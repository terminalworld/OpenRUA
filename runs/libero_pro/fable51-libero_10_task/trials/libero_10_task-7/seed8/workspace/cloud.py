#!/usr/bin/env python3
"""Project a camera's depth frame into world points; find clusters above the table.

Usage: python3 cloud.py <camera> [zmin] [zmax]
Saves <camera>_cloud.npz (xyz HxWx3 world, color) and prints clusters.
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
import cv2


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def grab(node, topic, typ, timeout=30.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
    return got["m"]


def main():
    cam = sys.argv[1]
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.83
    zmax = float(sys.argv[3]) if len(sys.argv) > 3 else 1.3
    rclpy.init()
    node = rclpy.create_node("cloud")
    tfs = {}
    node.create_subscription(TFMessage, "/tf",
                             lambda m: [tfs.setdefault(t.child_frame_id, t.transform) for t in m.transforms], 10)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while frame not in tfs and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfs[frame]
    br = CvBridge()
    depth = br.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(color_msg, "bgr8")
    H, W = depth.shape
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    pc = np.stack([X, Y, depth], -1)
    R = quat_to_R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    T = np.array([t.translation.x, t.translation.y, t.translation.z])
    xyz = pc @ R.T + T
    np.savez(f"{cam}_cloud.npz", xyz=xyz, color=color, depth=depth)
    valid = np.isfinite(depth) & (depth > 0)
    zs = xyz[..., 2][valid]
    hist, edges = np.histogram(zs, bins=60, range=(0, 3.2))
    print("z histogram (world):")
    for h, e in zip(hist, edges):
        if h > 200:
            print(f"  z~{e:.3f}: {h}")
    mask = valid & (xyz[..., 2] > zmin) & (xyz[..., 2] < zmax)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    print(f"clusters with z in ({zmin},{zmax}):")
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 15:
            continue
        m = lab == i
        p = xyz[m]
        c = color[m].mean(0)
        print(f"  #{i} area={stats[i, cv2.CC_STAT_AREA]} px_center=({cents[i][0]:.0f},{cents[i][1]:.0f}) "
              f"x=[{p[:,0].min():.3f},{p[:,0].max():.3f}] y=[{p[:,1].min():.3f},{p[:,1].max():.3f}] "
              f"z=[{p[:,2].min():.3f},{p[:,2].max():.3f}] mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f},{p[:,2].mean():.3f}) bgr={c.round(0)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
