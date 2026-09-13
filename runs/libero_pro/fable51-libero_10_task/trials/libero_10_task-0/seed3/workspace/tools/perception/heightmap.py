#!/usr/bin/env python3
"""Top-down height map from a camera's depth: python3 heightmap.py <camera> <out_prefix>
Saves <out_prefix>.npy (world points Nx3 above table) and prints object clusters."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2

def _grab(node, topic, msg_type, timeout=15.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got: raise SystemExit(f"no message on {topic}")
    return got["m"]

def main():
    cam, out = sys.argv[1], sys.argv[2]
    zmin = float(sys.argv[3]) if len(sys.argv) > 3 else 0.435
    rclpy.init(); node = rclpy.create_node("heightmap")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth = _grab(node, f"/{cam}/depth/image_raw", Image)
    info = _grab(node, f"/{cam}/color/camera_info", CameraInfo)
    color = _grab(node, f"/{cam}/color/image_raw", Image)
    C = np.frombuffer(color.data, dtype=np.uint8).reshape(color.height, color.width, -1)[:, :, :3]
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
    tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    vs, us = np.mgrid[0:depth.height, 0:depth.width]
    Z = D; ok = np.isfinite(Z) & (Z > 0)
    X = (us - cx) * Z / fx; Y = (vs - cy) * Z / fy
    P = np.stack([X, Y, Z], -1)[ok] @ R.T + tr
    col = C[ok]
    sel = (P[:, 2] > zmin) & (P[:, 2] < 0.75) & (np.abs(P[:, 0]) < 0.6) & (np.abs(P[:, 1]) < 0.6)
    P, col = P[sel], col[sel]
    np.save(out + ".npy", np.hstack([P, col]))
    # top-down raster: 2.5 mm cells over x,y in [-0.6,0.6]
    res = 0.0025; n = int(1.2 / res)
    img = np.zeros((n, n, 3), np.uint8); hm = np.zeros((n, n), np.float32)
    ix = ((P[:, 0] + 0.6) / res).astype(int).clip(0, n-1)
    iy = ((P[:, 1] + 0.6) / res).astype(int).clip(0, n-1)
    order = np.argsort(P[:, 2])
    img[n-1-iy[order], ix[order]] = col[order]  # y up, x right
    hm[n-1-iy[order], ix[order]] = P[order, 2]
    # grid lines every 0.1 m
    for k in range(0, n, int(0.1/res)):
        img[k, :] = (60, 60, 60); img[:, k] = (60, 60, 60)
    cv2.imwrite(out + ".png", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    np.save(out + "_hm.npy", hm)
    print("saved", out, "points:", len(P))

if __name__ == "__main__":
    main()
