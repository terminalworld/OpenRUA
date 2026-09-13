#!/usr/bin/env python3
"""Segment objects from a camera's depth image, in world coordinates.

Usage: python3 seg.py <camera> name:xmin,xmax,ymin,ymax,zmin [...]
Prints centroid, extents and top height of the points inside each box.
Saves <camera>_world.npy (HxWx3 world points).
"""
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from arm import quat_to_R


def cloud(cam, node=None):
    own = node is None
    if own:
        rclpy.init()
    n = node or rclpy.create_node("seg")
    buf = Buffer()
    tl = TransformListener(buf, n)
    got = {}
    n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
    n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
    frame = f"{cam}_optical_frame"
    while len(got) < 2 or not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(n, timeout_sec=0.2)
    d = got["d"]
    D = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
    k = got["i"].k
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R((q.x, q.y, q.z, q.w))
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    v, u = np.mgrid[0:d.height, 0:d.width]
    P = np.stack([(u - cx) * D / fx, (v - cy) * D / fy, D], -1) @ R.T + T
    tl.unregister()
    if own:
        rclpy.shutdown()
    return P


def region(P, name, xr, yr, zmin, zmax=2.0):
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = (X > xr[0]) & (X < xr[1]) & (Y > yr[0]) & (Y < yr[1]) & (Z > zmin) & (Z < zmax)
    if m.sum() == 0:
        print(name, "none")
        return None
    c = np.array([X[m].mean(), Y[m].mean()])
    print(f"{name}: n={m.sum()} center={np.round(c, 4)} x=[{X[m].min():.3f},{X[m].max():.3f}] "
          f"y=[{Y[m].min():.3f},{Y[m].max():.3f}] ztop={Z[m].max():.4f}")
    return c


if __name__ == "__main__":
    cam = sys.argv[1]
    P = cloud(cam)
    np.save(f"{cam}_world.npy", P)
    for spec in sys.argv[2:]:
        name, box = spec.split(":")
        vals = [float(x) for x in box.split(",")]
        region(P, name, vals[0:2], vals[2:4], vals[4], vals[5] if len(vals) > 5 else 2.0)
