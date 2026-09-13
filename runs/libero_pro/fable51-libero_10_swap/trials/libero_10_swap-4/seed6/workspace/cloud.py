#!/usr/bin/env python3
"""Project a camera's depth frame to world points; cluster things above the table.

Usage: python3 cloud.py <camera> [zmin]
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
from scipy import ndimage


def grab(node, topic, mt, timeout=20.0):
    got = {}
    sub = node.create_subscription(mt, topic, lambda m: got.setdefault("m", m), 1)
    import time
    t0 = time.time()
    while "m" not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_T(t):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4); T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def main():
    cam = sys.argv[1]
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.44
    rclpy.init()
    node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time
    t0 = time.time()
    while time.time() - t0 < 10 and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    T = quat_T(buf.lookup_transform("world", frame, rclpy.time.Time()))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    v, u = np.mgrid[0:H, 0:W]
    z = depth
    X = (u - cx) * z / fx; Y = (v - cy) * z / fy
    P = np.stack([X, Y, z, np.ones_like(z)], -1) @ T.T
    wx, wy, wz = P[..., 0], P[..., 1], P[..., 2]
    np.save(f"{cam}_world.npy", P[..., :3])
    mask = np.isfinite(z) & (wz > zmin) & (wz < 0.9) & (np.abs(wx) < 0.6) & (np.abs(wy) < 0.7)
    lab, n = ndimage.label(mask)
    print(f"{n} clusters above z={zmin}")
    for i in range(1, n + 1):
        m = lab == i
        if m.sum() < 15:
            continue
        us, vs = u[m], v[m]
        print(f"cluster {i}: npx={m.sum()} px u[{us.min()}-{us.max()}] v[{vs.min()}-{vs.max()}] "
              f"x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] "
              f"z[{wz[m].min():.3f},{wz[m].max():.3f}] centroid=({wx[m].mean():.3f},{wy[m].mean():.3f},{wz[m].mean():.3f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
