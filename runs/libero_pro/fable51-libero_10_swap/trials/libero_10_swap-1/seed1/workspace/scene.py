#!/usr/bin/env python3
"""Segment objects on the table from one camera's depth + color.

Usage: python3 scene.py <camera> [min_height_above_table=0.01]
Prints world-frame cluster centroids / bounding boxes and writes
<camera>_labels.png with cluster ids drawn on the color image.
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scipy import ndimage


def grab(node, topic, msg_type, timeout=20.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    hmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.01
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
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
    T = np.array([t.transform.translation.x, t.transform.translation.y,
                  t.transform.translation.z])
    br = CvBridge()
    depth = br.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(color_msg, "bgr8")
    H, W = depth.shape
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - cx) * depth / fx
    Y = (vs - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T  # world coords HxWx3
    valid = np.isfinite(depth) & (depth > 0)
    # table height: the most common z among valid points
    zs = P[..., 2][valid]
    hist, edges = np.histogram(zs, bins=400)
    table_z = edges[np.argmax(hist)] + (edges[1] - edges[0]) / 2
    print(f"camera {cam} at world {T.round(3)}; table_z ~ {table_z:.3f}")
    mask = valid & (P[..., 2] > table_z + hmin) & (P[..., 2] < table_z + 0.5)
    lab, n = ndimage.label(mask, structure=np.ones((3, 3)))
    out = color.copy()
    rows = []
    for i in range(1, n + 1):
        sel = lab == i
        if sel.sum() < 30:
            continue
        pts = P[sel]
        vv, uu = np.nonzero(sel)
        c = pts.mean(0)
        lo, hi = pts.min(0), pts.max(0)
        rows.append((i, int(uu.mean()), int(vv.mean()), c, lo, hi, sel.sum()))
        cv2.putText(out, str(i), (int(uu.mean()), int(vv.mean())),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.rectangle(out, (uu.min(), vv.min()), (uu.max(), vv.max()),
                      (0, 255, 0), 1)
    for i, u, v, c, lo, hi, npx in rows:
        print(f"[{i:2d}] px=({u},{v}) n={npx:5d} "
              f"center=({c[0]:.3f},{c[1]:.3f},{c[2]:.3f}) "
              f"x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] "
              f"z[{lo[2]:.3f},{hi[2]:.3f}]")
    cv2.imwrite(f"{cam}_labels.png", out)
    np.save(f"{cam}_P.npy", P)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
