#!/usr/bin/env python3
"""Scene measurements from the birdview depth camera (top-down)."""
import numpy as np, rclpy
from sensor_msgs.msg import Image

FX = 579.4112549695428
HINGE = np.array([-0.290, -0.365])   # microwave door hinge (world x,y)
BOX = dict(x=(-0.293, 0.058), y=(-0.35, -0.09), top=1.107, floor=0.944,
           open_x=(-0.255, -0.046))  # cavity opening on the -y face


def birdview_cloud(node=None):
    own = node is None
    if own:
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node("bird")
    got = []
    sub = node.create_subscription(Image, "/birdview/depth/image_raw", got.append, 1)
    while not got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    m = got[0]
    D = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
    vv, uu = np.mgrid[0:m.height, 0:m.width]
    X = -0.2 + (vv - 240) * D / FX
    Y = (uu - 320) * D / FX
    Z = 3.0 - D
    if own:
        node.destroy_node()
    return X, Y, Z


def door_state(X, Y, Z):
    """Door opening angle (deg, 0 = closed along +x) and tip point."""
    m = (Z > 0.95) & (Z < 1.15) & (Y < -0.355) & (X > -0.7) & (X < 0.2)
    # exclude the robot arm: keep points within 0.30 m of the hinge
    r = np.hypot(X - HINGE[0], Y - HINGE[1])
    m &= (r < 0.30) & (r > 0.03)
    if m.sum() < 20:
        return None, None
    pts = np.c_[X[m], Y[m]]
    d = pts - HINGE
    far = d[np.argsort(np.hypot(*d.T))[-40:]].mean(0)
    ang = np.degrees(np.arctan2(-far[1], far[0]))  # cw from +x
    return ang, HINGE + far


def mug_state(X, Y, Z, region=None):
    """Yellow/white mug rim circle (centre x,y, radius, rim z)."""
    if region is None:
        region = (X > -0.15, X < 0.2, Y > -0.08, Y < 0.15)
    m = np.ones_like(Z, bool)
    for c in region:
        m &= c
    m &= (Z > 0.93) & (Z < 1.2)
    if m.sum() < 20:
        return None
    ztop = np.percentile(Z[m], 98)
    rim = m & (Z > ztop - 0.012)
    pts = np.c_[X[rim], Y[rim]]
    A = np.c_[2 * pts, np.ones(len(pts))]; b = (pts ** 2).sum(1)
    c = np.linalg.lstsq(A, b, rcond=None)[0]
    cx, cy = c[0], c[1]; r = np.sqrt(c[2] + cx * cx + cy * cy)
    return np.array([cx, cy]), r, ztop


if __name__ == "__main__":
    X, Y, Z = birdview_cloud()
    ang, tip = door_state(X, Y, Z)
    print(f"door angle {ang:.1f} deg, tip {tip.round(3)}")
    ms = mug_state(X, Y, Z)
    print("mug", None if ms is None else (ms[0].round(3), round(ms[1], 3), round(ms[2], 3)))
    ms2 = mug_state(X, Y, Z, region=(X > -0.30, X < 0.06, Y > -0.35, Y < -0.09))
    print("mug-in-cavity-region", None if ms2 is None else (ms2[0].round(3), round(ms2[1], 3), round(ms2[2], 3)))
