"""MuJoCo: what every engine on it shares, over the raw model and data.

robosuite (through its MjSim) and dm_control (VLABench's physics) wrap
MuJoCo differently, but the world they expose is the same MjModel and
MjData: joint addressing by qpos and dof address, body and site poses,
camera poses and vertical fields of view, forward kinematics on a
scratch MjData. This base holds that knowledge once; an engine derives
from it with two accessors (``_model`` and ``_data``, the raw structs,
read anew every call because a hard reset may replace them) and adds
what its wrapper decides: rendering, the action layout, effort, wrench.

Facts fixed here: MuJoCo cameras look along -Z, so the optical frame is
a rotation of pi about X; quaternions are w x y z; a camera's intrinsic
matrix follows from its fovy and the requested image size (the formula
robosuite's helper uses).
"""

from __future__ import annotations

import math

import numpy as np

from .frames import GL2OPTICAL


class MuJoCo:
    """The shared half of a MuJoCo engine. Subclasses define ``_model``
    and ``_data`` properties and the rest of ``ENGINE_INTERFACE``."""

    def __init__(self):
        self._arms: list = []
        self._scratch = None  # (model, MjData) FK scratch (lazy)
        self._hand_ids: dict = {}

    # ---------------------------------------------------------------- world
    def _name(self, kind, i: int) -> str:
        import mujoco

        return mujoco.mj_id2name(self._model, kind, i) or ""

    def _id(self, kind, name: str) -> int:
        import mujoco

        i = mujoco.mj_name2id(self._model, kind, name)
        if i < 0:
            raise KeyError(name)
        return i

    def joints(self) -> list[tuple[str, int, int]]:
        import mujoco

        m = self._model
        return [(self._name(mujoco.mjtObj.mjOBJ_JOINT, j), int(m.jnt_qposadr[j]), int(m.jnt_dofadr[j]))
                for j in range(m.njnt)]

    def time(self) -> float:
        return float(self._data.time)

    def qpos(self) -> np.ndarray:
        return self._data.qpos

    def qvel(self) -> np.ndarray:
        return self._data.qvel

    def effort(self) -> np.ndarray:
        return self._data.qfrc_actuator

    def body_pose(self, name: str):
        import mujoco

        bid = self._id(mujoco.mjtObj.mjOBJ_BODY, name)
        d = self._data
        return d.xpos[bid], d.xquat[bid], d.xmat[bid].reshape(3, 3)

    def site_pos(self, name: str):
        import mujoco

        return self._data.site_xpos[self._id(mujoco.mjtObj.mjOBJ_SITE, name)]

    # -------------------------------------------------------------- cameras
    def camera_names(self) -> list[str]:
        import mujoco

        return [self._name(mujoco.mjtObj.mjOBJ_CAMERA, c) for c in range(self._model.ncam)]

    def camera_pose(self, name: str):
        import mujoco

        cid = self._id(mujoco.mjtObj.mjOBJ_CAMERA, name)
        d = self._data
        return d.cam_xpos[cid], d.cam_xmat[cid].reshape(3, 3) @ GL2OPTICAL

    def camera_size(self, name: str, width: int, height: int):
        return width, height  # MuJoCo renders any camera at any size

    def intrinsics(self, name: str, width: int, height: int) -> np.ndarray:
        import mujoco

        fovy = math.radians(float(self._model.cam_fovy[self._id(mujoco.mjtObj.mjOBJ_CAMERA, name)]))
        f = height / 2.0 / math.tan(fovy / 2.0)
        return np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]])

    # ------------------------------------------------------------------- FK
    def fk(self, arm: int, q: np.ndarray | None = None):
        """FK of the arm's hand body on a scratch MjData; q=None reads the
        live state. The scratch is rebuilt when the model changes."""
        import mujoco

        a = self._arms[arm]
        model = self._model
        if self._scratch is None or self._scratch[0] is not model:
            self._scratch = (model, mujoco.MjData(model))
            self._hand_ids = {}
        if arm not in self._hand_ids:
            self._hand_ids[arm] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, a.hand_body)
        d = self._scratch[1]
        d.qpos[:] = self._data.qpos
        if q is not None:
            d.qpos[a.qadrs] = q
        mujoco.mj_kinematics(model, d)
        hid = self._hand_ids[arm]
        return d.xpos[hid].copy(), d.xmat[hid].reshape(3, 3).copy()
