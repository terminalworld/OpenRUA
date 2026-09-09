"""calvin_env (PyBullet) engine: the bridge's view of CALVIN's play table.

The env is calvin_env's ``PlayTableSimEnv``: a PyBullet client
(``env.p``, ``env.cid``), a ``Robot`` (the Panda body ``robot_uid``
with its joint ids and motor settings) and a list of cameras
(``StaticCamera``, ``GripperCamera``) that render through PyBullet's
``getCameraImage`` (TinyRenderer on the CPU). The loader hands the
bridge an episode object whose ``env`` is the live env and whose
``step`` drives one control tick (the robot's position motors held for
the env's ``action_repeat`` physics steps).

Engine facts kept here: PyBullet joint state is per joint index (fixed
joints included, so the joint list skips them); link frames come from
``getLinkState`` world link frame, orientations as x y z w; a camera's
view matrix is column-major OpenGL (looking along -Z), its projection
a vertical field of view; forward kinematics runs on a private DIRECT
client that loads the same URDF at the same base pose, so the world
never moves for a jacobian.
"""

from __future__ import annotations

import math

import numpy as np

from .maniskill import _GL2OPTICAL, _quat_to_mat

_FIXED = 4  # pybullet.JOINT_FIXED


def _xyzw_to_wxyz(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([w, x, y, z])


class _Arm:
    def __init__(self, qadrs: np.ndarray, hand_body: str):
        self.qadrs = np.asarray(qadrs)
        self.hand_body = hand_body


class Calvin:
    def __init__(self, env, cfg: dict):
        self._episode = env
        self._arms: list[_Arm] = []
        self._scratch = None  # (client, uid) for FK
        p, cid, uid = self._p, self._cid, self._uid
        self._n = p.getNumJoints(uid, physicsClientId=cid)
        self._links = {p.getBodyInfo(uid, physicsClientId=cid)[0].decode(): -1}
        self._types = {}
        self._names = {}
        for i in range(self._n):
            info = p.getJointInfo(uid, i, physicsClientId=cid)
            self._names[i] = info[1].decode()
            self._types[i] = info[2]
            self._links[info[12].decode()] = i

    @property
    def _env(self):
        return self._episode.env

    @property
    def _p(self):
        return self._env.p

    @property
    def _cid(self):
        return self._env.cid

    @property
    def _uid(self):
        return self._env.robot.robot_uid

    # ---------------------------------------------------------------- world
    def joints(self) -> list[tuple[str, int, int]]:
        return [(self._names[i], i, i) for i in range(self._n) if self._types[i] != _FIXED]

    def time(self) -> float:
        return float(self._episode.steps) * self._episode.physics_dt

    def _states(self):
        return self._p.getJointStates(self._uid, list(range(self._n)), physicsClientId=self._cid)

    def qpos(self) -> np.ndarray:
        return np.array([s[0] for s in self._states()], dtype=float)

    def qvel(self) -> np.ndarray:
        return np.array([s[1] for s in self._states()], dtype=float)

    def effort(self) -> np.ndarray:
        return np.array([s[3] for s in self._states()], dtype=float)

    def body_pose(self, name: str):
        if name not in self._links:
            raise KeyError(name)
        idx = self._links[name]
        if idx < 0:
            pos, orn = self._p.getBasePositionAndOrientation(self._uid, physicsClientId=self._cid)
        else:
            pos, orn = self._p.getLinkState(self._uid, idx, physicsClientId=self._cid)[4:6]
        q = _xyzw_to_wxyz(orn)
        return np.asarray(pos, dtype=float), q, _quat_to_mat(q)

    def site_pos(self, name: str):
        raise KeyError(name)

    def objects(self) -> dict:
        info = self._env.get_info().get("scene_info", {})
        out = {}
        for name, obj in info.get("movable_objects", {}).items():
            if "current_pos" in obj:
                out[f"{name}_pos"] = [float(v) for v in obj["current_pos"]]
            if "current_orn" in obj:
                out[f"{name}_quat"] = [float(v) for v in obj["current_orn"]]
        return out

    # -------------------------------------------------------------- cameras
    def _camera(self, name: str):
        for cam in self._env.cameras:
            if cam.name == name:
                return cam
        raise KeyError(name)

    def camera_names(self) -> list[str]:
        return [cam.name for cam in self._env.cameras]

    def camera_size(self, name: str, width: int, height: int):
        cam = self._camera(name)
        return int(cam.width), int(cam.height)

    def _view(self, cam) -> np.ndarray:
        if hasattr(cam, "viewMatrix"):  # static: fixed at creation
            v = cam.viewMatrix
        else:  # gripper: follows the hand, computed the way its render does
            p = self._p
            pos, orn = p.getLinkState(self._uid, cam.gripper_cam_link, physicsClientId=self._cid)[:2]
            rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
            v = p.computeViewMatrix(pos, np.asarray(pos) + rot[:, 1], -rot[:, 2])
        return np.array(v, dtype=float).reshape(4, 4).T  # column-major -> row-major

    def camera_pose(self, name: str):
        cam2world = np.linalg.inv(self._view(self._camera(name)))
        return cam2world[:3, 3], cam2world[:3, :3] @ _GL2OPTICAL

    def render(self, name: str, width: int, height: int, depth: bool = False):
        rgb, d = self._camera(name).render()
        rgb = np.asarray(rgb).astype(np.uint8)
        return (rgb, np.asarray(d, dtype=np.float32)) if depth else rgb

    def intrinsics(self, name: str, width: int, height: int) -> np.ndarray:
        cam = self._camera(name)
        f = cam.height / (2.0 * math.tan(math.radians(cam.fov) / 2.0))
        return np.array([[f / cam.aspect, 0.0, cam.width / 2.0],
                         [0.0, f, cam.height / 2.0],
                         [0.0, 0.0, 1.0]])

    # ------------------------------------------------------------ actuation
    def control_dt(self) -> float:
        return self._episode.control_dt

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
               base_vel: np.ndarray | None) -> dict:
        """Absolute joint targets for the arm's joint indices, and the
        gripper as CALVIN states it (1 open, -1 close)."""
        a = self._arms[arm]
        q = self.qpos()[a.qadrs] + dq
        return {"joints": [int(i) for i in a.qadrs], "targets": q, "gripper": -1 if grips[arm] > 0 else 1}

    def step(self, action) -> None:
        self._episode.step(action)

    def fk(self, arm: int, q: np.ndarray | None = None):
        import pybullet as pb
        from pybullet_utils import bullet_client as bc

        a = self._arms[arm]
        robot = self._env.robot
        if self._scratch is None:
            client = bc.BulletClient(connection_mode=pb.DIRECT)
            client.setAdditionalSearchPath(self._env.scene.data_path.as_posix())
            uid = client.loadURDF(fileName=robot.filename, basePosition=robot.base_position,
                                  baseOrientation=robot.base_orientation, useFixedBase=True)
            self._scratch = (client, uid)
        client, uid = self._scratch
        full = self.qpos()
        if q is not None:
            full[a.qadrs] = q
        for i in range(self._n):
            if self._types[i] != _FIXED:
                client.resetJointState(uid, i, float(full[i]))
        idx = self._links[a.hand_body]
        pos, orn = client.getLinkState(uid, idx, computeForwardKinematics=1)[4:6]
        return np.asarray(pos, dtype=float), _quat_to_mat(_xyzw_to_wxyz(orn))


ENGINE = Calvin
