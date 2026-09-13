"""Helpers: load a saved depth .npy for a camera and produce world-frame points."""
import numpy as np

FX = 579.4112549695428
CX, CY = 320.0, 240.0

CAMS = {
    # cam: (translation, quaternion xyzw) of <cam>_optical_frame in world
    "birdview": ((-0.2, 0.0, 3.0), (0.7071067811865476, 0.7071067811865477, 0.0, 0.0)),
    "frontview": ((1.0, 0.0, 1.48), (0.5608418947374367, 0.5608418947374367, -0.4306464548876746, -0.4306464548876746)),
}


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def world_points(cam, npy=None):
    D = np.load(npy or f"{cam}_depth.npy")
    H, W = D.shape
    v, u = np.mgrid[0:H, 0:W]
    pc = np.stack([(u - CX) * D / FX, (v - CY) * D / FX, D], axis=-1)
    t, q = CAMS[cam]
    R = quat_R(*q)
    pw = pc @ R.T + np.array(t)
    return pw[..., 0], pw[..., 1], pw[..., 2], D
