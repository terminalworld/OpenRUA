import numpy as np


def load(cam):
    z = np.load(f"{cam}.npz")
    return z["color"], z["depth"], z["K"], z["T"]


def px(cam, u, v, verbose=True):
    _, d, K, T = load(cam)
    zc = float(d[v, u])
    if not np.isfinite(zc) or zc <= 0:
        raise ValueError(f"bad depth {zc}")
    p = np.array([(u - K[0, 2]) * zc / K[0, 0], (v - K[1, 2]) * zc / K[1, 1], zc, 1.0])
    w = T @ p
    if verbose:
        print(f"{cam} ({u},{v}) depth={zc:.3f} -> world {w[0]:.4f} {w[1]:.4f} {w[2]:.4f}")
    return w[:3]


def cloud(cam):
    """full point cloud in world frame, shape (H,W,3)"""
    _, d, K, T = load(cam)
    H, W = d.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - K[0, 2]) * d / K[0, 0]
    Y = (vs - K[1, 2]) * d / K[1, 1]
    P = np.stack([X, Y, d, np.ones_like(d)], -1)
    return (P @ T.T)[..., :3]


def world2px(cam, xyz):
    _, d, K, T = load(cam)
    Ti = np.linalg.inv(T)
    p = Ti @ np.array([*xyz, 1.0])
    u = K[0, 0] * p[0] / p[2] + K[0, 2]
    v = K[1, 1] * p[1] / p[2] + K[1, 2]
    return u, v
