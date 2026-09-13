#!/usr/bin/env python3
"""Fixed-camera geometry from /workspace/cam_extrinsics.json (pose of the
optical frame in world; optical: x right, y down, z forward)."""
import json
import numpy as np
import sys
sys.path.insert(0, "/workspace/tools")
from robot import quat_to_mat

EXT = json.load(open("/workspace/cam_extrinsics.json"))
F = {"default": 579.4112549695428, "robot0_eye_in_hand": 312.77408948188935}
CX, CY = 320.0, 240.0


def cam_pose(name):
    e = EXT[f"{name}_optical_frame"]
    return np.array(e["t"]), quat_to_mat(e["q"])


def cloud(name, depth, pos=None, R=None):
    """(H,W) depth -> (H,W,3) world points."""
    if pos is None:
        pos, R = cam_pose(name)
    f = F.get(name, F["default"])
    H, W = depth.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - CX) / f * depth
    Y = (vs - CY) / f * depth
    P = np.stack([X, Y, depth], -1)
    return pos + P @ R.T


def project(name, pts):
    """world points (N,3) -> pixel (N,2) and depth (N,)"""
    pos, R = cam_pose(name)
    f = F.get(name, F["default"])
    Pc = (np.asarray(pts) - pos) @ R
    u = Pc[:, 0] / Pc[:, 2] * f + CX
    v = Pc[:, 1] / Pc[:, 2] * f + CY
    return np.stack([u, v], -1), Pc[:, 2]
