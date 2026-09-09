"""Frame conventions every engine converts to: shared, engine-free.

Quaternions arrive as w x y z (the order the interface fixes); a camera
that looks along -Z (OpenGL, which MuJoCo and SAPIEN both use) becomes
a ROS optical frame (+Z forward, REP 103/104) by a rotation of pi about
X.
"""

from __future__ import annotations

import numpy as np

# OpenGL cameras look along -Z; ROS optical frames look along +Z: rotate
# pi about X to convert.
GL2OPTICAL = np.diag([1.0, -1.0, -1.0])


def quat_to_mat(q) -> np.ndarray:
    """(w, x, y, z) -> rotation matrix."""
    w, x, y, z = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
