#!/usr/bin/env python3
"""Grab the eye-in-hand colour+depth, save PNGs, and print world-frame
segments in a height band. Camera pose from FK (not TF, which can be stale).

Usage: python3 eyescan.py [zlo zhi] [tag]
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from robot import Robot, quat_R, quat_mul

CAM = "robot0_eye_in_hand"
# camera optical frame in panda_hand: from tf_static
CAM_T = np.array([0.050, 0.0, -0.001])
CAM_Q = np.array([0.0, 0.0, 0.7071068, 0.7071068])


def grab(node, topic, typ):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def cam_pose(r):
    p, q = r.fk()
    R = quat_R(*q)
    Rc = quat_R(*quat_mul(q, CAM_Q))
    return p + R @ CAM_T, Rc


def cloud(r, depth, info):
    """World-frame point cloud (H,W,3) for a depth image."""
    T, Rc = cam_pose(r)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - cx) * depth / fx
    Y = (vs - cy) * depth / fy
    return np.stack([X, Y, depth], -1) @ Rc.T + T


def snap(r, tag="eye"):
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(r.node, f"/{CAM}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(r.node, f"/{CAM}/color/image_raw", Image), "bgr8")
    info = grab(r.node, f"/{CAM}/color/camera_info", CameraInfo)
    cv2.imwrite(f"{tag}.png", color)
    P = cloud(r, depth, info)
    return color, depth, info, P


def segments(P, depth, color, zlo, zhi, min_area=40):
    valid = np.isfinite(depth) & (depth > 0)
    Z = P[..., 2]
    mask = valid & (Z > zlo) & (Z < zhi)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    out = []
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < min_area:
            continue
        m = lab == i
        pts = P[m]
        col = color[m].mean(0)[::-1]
        u, v = cents[i]
        out.append(dict(px=(u, v), area=int(area), c=pts.mean(0), zmax=pts[:, 2].max(),
                        zmin=pts[:, 2].min(), xr=(pts[:, 0].min(), pts[:, 0].max()),
                        yr=(pts[:, 1].min(), pts[:, 1].max()), rgb=col.astype(int), mask=m))
    return out


if __name__ == "__main__":
    zlo, zhi = (float(sys.argv[1]), float(sys.argv[2])) if len(sys.argv) >= 3 else (0.5, 0.7)
    tag = sys.argv[3] if len(sys.argv) > 3 else "eye"
    r = Robot("eyescan")
    color, depth, info, P = snap(r, tag)
    T, Rc = cam_pose(r)
    print("cam at", T.round(4))
    v = np.isfinite(depth) & (depth > 0)
    print("z range in view", P[..., 2][v].min().round(3), P[..., 2][v].max().round(3))
    for s in segments(P, depth, color, zlo, zhi):
        print(f"px=({s['px'][0]:.0f},{s['px'][1]:.0f}) area={s['area']} c={s['c'].round(3)} "
              f"z=({s['zmin']:.3f},{s['zmax']:.3f}) xr=({s['xr'][0]:.3f},{s['xr'][1]:.3f}) "
              f"yr=({s['yr'][0]:.3f},{s['yr'][1]:.3f}) rgb={s['rgb']}")
    # table height estimate: most common z
    zs = P[..., 2][v]
    hist, edges = np.histogram(zs, bins=200)
    print("mode z", edges[hist.argmax()].round(3))
