#!/usr/bin/env python3
"""Point-cloud scan of a camera: cluster points above the table, print
world centroid, extents and mean color per cluster.
Usage: python3 scan.py <camera> [table_z=0.43]
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scipy import ndimage


def grab(node, topic, T, timeout=15.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
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
    table_z = float(sys.argv[2]) if len(sys.argv) > 2 else 0.43
    rclpy.init()
    node = rclpy.create_node("scan")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "rgb8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + tr
    valid = np.isfinite(depth) & (depth > 0)
    above = valid & (P[..., 2] > table_z) & (P[..., 2] < table_z + 0.35)
    # exclude robot: anything within 0.2 m xy of base (-0.51, 0) roughly and its arm above
    lab, n = ndimage.label(above)
    print(f"{cam}: {n} clusters above z={table_z}")
    for i in range(1, n + 1):
        m = lab == i
        if m.sum() < 30:
            continue
        pts = P[m]
        c = color[m].mean(0)
        uu, vv = u[m].mean(), v[m].mean()
        print(f"  #{i} n={m.sum():5d} px=({uu:5.0f},{vv:5.0f}) "
              f"cen=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f},{pts[:,2].mean():.3f}) "
              f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
              f"z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] rgb=({c[0]:.0f},{c[1]:.0f},{c[2]:.0f})")
    np.save(f"{cam}_P.npy", P)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
