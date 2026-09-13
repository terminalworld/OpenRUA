#!/usr/bin/env python3
"""Build a world-frame point cloud from a saved depth .npy + live intrinsics/TF,
then report the extents of objects above the table in a region.
Usage: cloud.py <cam> <depth.npy> xmin xmax ymin ymax"""
import sys, time
import numpy as np, rclpy
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
from px import grab, tfmat


def main():
    cam, npy = sys.argv[1], sys.argv[2]
    xmin, xmax, ymin, ymax = map(float, sys.argv[3:7])
    rclpy.init(); node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    end = time.time() + 10
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    Tc = tfmat(buf.lookup_transform("world", frame, rclpy.time.Time()))
    rclpy.shutdown()
    d = np.load(npy)
    h, w = d.shape
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    vs, us = np.mgrid[0:h, 0:w]
    z = d
    pts = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z, np.ones_like(z)], -1).reshape(-1, 4)
    ok = np.isfinite(pts[:, 2]) & (pts[:, 2] > 0.05)
    P = (Tc @ pts[ok].T).T[:, :3]
    sel = (P[:, 0] > xmin) & (P[:, 0] < xmax) & (P[:, 1] > ymin) & (P[:, 1] < ymax)
    Q = P[sel]
    print("n", len(Q), "z range", Q[:, 2].min().round(3), Q[:, 2].max().round(3))
    table = np.median(Q[:, 2])
    print("table z (median)", table.round(3))
    for lo in np.arange(0.905, 1.10, 0.01):
        s = Q[(Q[:, 2] >= lo) & (Q[:, 2] < lo + 0.01)]
        if len(s) < 3:
            continue
        print(f"z[{lo:.3f},{lo+0.01:.3f}) n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] "
              f"y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")


if __name__ == "__main__":
    main()
