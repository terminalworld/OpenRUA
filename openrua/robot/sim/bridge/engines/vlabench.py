"""VLABench (dm_control on MuJoCo) engine: the bridge's view of a VLABench env.

The env is VLABench's ``LM4ManipDMEnv``, a dm_control composer
environment: ``env.physics`` wraps the MjModel and MjData the shared
MuJoCo base reads (joints, poses, cameras, FK); ``env.robot`` is the
arm (its seven joints in order, two finger joints); ``env.step`` takes
the seven joint targets and the two finger targets and advances one
control step (dm_control renders upright images and metric depth
through ``physics.render``).

Engine facts kept here: the action is absolute joint positions; the
fingers open at 0.04 and close at 0; dm_control's physics is rebuilt on
some resets, so the raw structs are read through the env every call.
"""

from __future__ import annotations

import numpy as np

from .mujoco import MuJoCo

_OPEN, _CLOSED = 0.04, 0.0


class _Arm:
    def __init__(self, qadrs: np.ndarray, hand_body: str):
        self.qadrs = np.asarray(qadrs)
        self.hand_body = hand_body


class VLABench(MuJoCo):
    def __init__(self, env, cfg: dict):
        super().__init__()
        self._env = env

    @property
    def _physics(self):
        return self._env.physics

    @property
    def _model(self):
        return self._physics.model.ptr

    @property
    def _data(self):
        return self._physics.data.ptr

    # ---------------------------------------------------------------- world
    def objects(self) -> dict:
        return {}  # the task's entities are not exposed by name in one place

    # -------------------------------------------------------------- cameras
    def render(self, name: str, width: int, height: int, depth: bool = False):
        rgb = np.asarray(self._physics.render(height=height, width=width, camera_id=name)).astype(np.uint8)
        if not depth:
            return rgb
        d = np.asarray(self._physics.render(height=height, width=width, camera_id=name, depth=True))
        return rgb, d.astype(np.float32)

    # ------------------------------------------------------------ actuation
    def control_dt(self) -> float:
        return float(self._env.control_timestep())

    def mobile(self) -> bool:
        return False

    def bind_arms(self, arms: list[tuple[np.ndarray, str]]) -> None:
        self._arms = [_Arm(qadrs, hand_body) for qadrs, hand_body in arms]

    def has_gripper(self, arm: int) -> bool:
        return True

    def ee_wrench(self, arm: int):
        return np.zeros(3), np.zeros(3)

    def apply_tuning(self) -> None:
        pass

    def action(self, arm: int, dq: np.ndarray, grips: list[float],
               base_vel: np.ndarray | None) -> np.ndarray:
        """Seven absolute joint targets then the two finger targets, the
        layout VLABench's step takes (bridge -1 open / +1 close)."""
        a = self._arms[arm]
        q = np.asarray(self._data.qpos[a.qadrs], dtype=float) + dq
        finger = _CLOSED if grips[arm] > 0 else _OPEN
        return np.concatenate([q, [finger, finger]])

    def step(self, action) -> None:
        self._env.step(action)


ENGINE = VLABench
