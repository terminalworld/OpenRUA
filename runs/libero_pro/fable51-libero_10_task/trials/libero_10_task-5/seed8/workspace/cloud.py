#!/usr/bin/env python3
"""Grab depth+intrinsics+TF for a camera, save world-frame point cloud as
<cam>_cloud.npy (N x 3) and print a top-down height map for a region.

Usage: python3 cloud.py <cam> [xmin xmax ymin ymax [res]]
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, mt, timeout=30.0):
    got = {}
    sub = node.create_subscription(mt, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
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
    node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    d = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    v, u = np.mgrid[0:depth.height, 0:depth.width]
    z = d
    X = (u - cx) * z / fx; Y = (v - cy) * z / fy
    P = np.stack([X, Y, z], -1).reshape(-1, 3)
    ok = np.isfinite(P[:, 2]) & (P[:, 2] > 0)
    W = (R @ P[ok].T).T + T
    np.save(f"{cam}_cloud.npy", W)
    uv = np.stack([u.reshape(-1)[ok], v.reshape(-1)[ok]], -1)
    np.save(f"{cam}_cloud_uv.npy", uv)
    print("saved", W.shape, "cam at", T)
    if len(sys.argv) >= 6:
        xmin, xmax, ymin, ymax = map(float, sys.argv[2:6])
        res = float(sys.argv[6]) if len(sys.argv) > 6 else 0.01
        sel = (W[:, 0] >= xmin) & (W[:, 0] < xmax) & (W[:, 1] >= ymin) & (W[:, 1] < ymax)
        S = W[sel]
        nx = int(round((xmax - xmin) / res)); ny = int(round((ymax - ymin) / res))
        H = np.full((nx, ny), np.nan)
        ix = ((S[:, 0] - xmin) / res).astype(int).clip(0, nx - 1)
        iy = ((S[:, 1] - ymin) / res).astype(int).clip(0, ny - 1)
        for a, b, zz in zip(ix, iy, S[:, 2]):
            if np.isnan(H[a, b]) or zz > H[a, b]:
                H[a, b] = zz
        print("rows = x from %.2f (top) to %.2f; cols = y from %.2f (left) to %.2f; cell=%.2fm; value=max z*100 (cm)" % (xmin, xmax, ymin, ymax, res))
        print("      " + " ".join(f"{ymin + j * res:5.2f}"[-3:] for j in range(ny)))
        for i in range(nx):
            row = " ".join(("  ." if np.isnan(H[i, j]) else f"{H[i, j] * 100:3.0f}") for j in range(ny))
            print(f"{xmin + i * res:5.2f} {row}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
