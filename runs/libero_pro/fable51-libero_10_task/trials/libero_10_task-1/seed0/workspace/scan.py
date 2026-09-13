#!/usr/bin/env python3
"""Cluster objects above the table from a camera's depth frame.

Usage: python3 scan.py <camera> [min_height_m]
Prints world-frame cluster centroids, extents, and top heights; also
saves <camera>_labels.png for visual cross-reference.
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, mt, timeout=20.0):
    got = {}
    sub = node.create_subscription(mt, topic, lambda m: got.setdefault("m", m), 1)
    import time
    t0 = time.time()
    while "m" not in got and time.time() - t0 < timeout:
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
    minh = float(sys.argv[2]) if len(sys.argv) > 2 else 0.01
    rclpy.init()
    node = rclpy.create_node("scan")
    buf = Buffer(); TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(color_msg, "bgr8")
    frame = f"{cam}_optical_frame"
    import time
    t0 = time.time()
    while time.time() - t0 < 10 and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    z = depth
    X = (us - cx) * z / fx
    Y = (vs - cy) * z / fy
    P = np.stack([X, Y, z], -1).reshape(-1, 3) @ R.T + T
    P = P.reshape(H, W, 3)
    valid = np.isfinite(z) & (z > 0)
    # table height = mode of world z among valid points
    zs = P[..., 2][valid]
    zs = zs[zs > 0.2]  # ignore the floor
    hist, edges = np.histogram(zs, bins=400)
    table_z = edges[np.argmax(hist)] + (edges[1] - edges[0]) / 2
    print(f"camera {cam} at world {T.round(3)}; table_z~{table_z:.3f}")
    mask = valid & (P[..., 2] > table_z + minh)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    out = color.copy()
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 30:
            continue
        m = lab == i
        pts = P[m]
        mn, mx = pts.min(0), pts.max(0)
        c = pts.mean(0)
        u, v = cents[i]
        print(f"#{i:2d} px=({u:5.0f},{v:5.0f}) area={area:5d} "
              f"centroid=({c[0]:.3f},{c[1]:.3f}) xy-extent=({mn[0]:.3f}..{mx[0]:.3f}, {mn[1]:.3f}..{mx[1]:.3f}) "
              f"top_z={mx[2]:.3f} (h={mx[2]-table_z:.3f})")
        x0, y0, w, h = stats[i, :4]
        cv2.rectangle(out, (x0, y0), (x0 + w, y0 + h), (0, 255, 0), 1)
        cv2.putText(out, str(i), (x0, y0 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    cv2.imwrite(f"{cam}_labels.png", out)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
