"""RoboTwin 2.0 (SAPIEN 3) engine: the bridge's view of a RoboTwin task.

RoboTwin has no gym env: a task object (``envs/<task>.py``, a
``Base_Task``) owns a SAPIEN scene, a ``Robot`` holding the dual-arm
articulation (one entity, both arms and the base in it) and a
``Camera`` set (head, front, left and right wrist). The loader hands
the bridge an episode object whose ``task`` attribute is the live task
and whose ``step`` drives one control tick; this engine reads through
it every time, because a reset builds a fresh task.

Engine facts kept here: joint targets go through the robot's own
``set_arm_joints`` / ``set_gripper`` (gripper value 0 closed .. 1
open); camera renders come from the task's ``Camera`` set after the
wrist cameras follow the arms; SAPIEN poses are (p, q) with q as
w x y z; ``cam2world_gl`` is the OpenGL convention; pinocchio gives
link poses in the robot's root frame.
"""

from __future__ import annotations

import numpy as np

from .maniskill import _GL2OPTICAL, _quat_to_mat


class _Arm:
    def __init__(self, tag: str, qadrs: np.ndarray, hand_body: str):
        self.tag = tag  # "left" | "right", RoboTwin's own labels
        self.qadrs = np.asarray(qadrs)
        self.hand_body = hand_body
        self.hand_index = None


class RoboTwin:
    def __init__(self, env, cfg: dict):
        self._env = env  # the loader's episode: .task, .step, .steps
        self._arms: list[_Arm] = []
        self._pinocchio = (None, None)  # (task id, model)

    @property
    def _task(self):
        return self._env.task

    @property
    def _entity(self):
        return self._task.robot.left_entity  # both arms live in it

    # ---------------------------------------------------------------- world
    def joints(self) -> list[tuple[str, int, int]]:
        return [(j.get_name(), i, i) for i, j in enumerate(self._entity.get_active_joints())]

    def time(self) -> float:
        return float(self._env.steps) * self._env.physics_dt

    def qpos(self) -> np.ndarray:
        return np.asarray(self._entity.get_qpos(), dtype=float)

    def qvel(self) -> np.ndarray:
        return np.asarray(self._entity.get_qvel(), dtype=float)

    def effort(self) -> np.ndarray:
        return np.asarray(self._entity.get_qf(), dtype=float)

    def _pose_of(self, name: str):
        for link in self._entity.get_links():
            if link.get_name() == name:
                return link.get_pose()
        for actor in self._task.scene.get_all_actors():
            if actor.get_name() == name:
                return actor.get_pose()
        raise KeyError(name)

    def body_pose(self, name: str):
        pose = self._pose_of(name)
        p, q = np.asarray(pose.p, dtype=float), np.asarray(pose.q, dtype=float)
        return p, q, _quat_to_mat(q)

    def site_pos(self, name: str):
        raise KeyError(name)

    def objects(self) -> dict:
        out = {}
        for actor in self._task.scene.get_all_actors():
            pose = actor.get_pose()
            out[f"{actor.get_name()}_pos"] = [float(v) for v in pose.p]
            out[f"{actor.get_name()}_quat"] = [float(v) for v in pose.q]
        return out

    # -------------------------------------------------------------- cameras
    def _cameras(self):
        return self._task.cameras

    def camera_names(self) -> list[str]:
        return list(self._cameras().get_config())

    def _config(self, name: str) -> dict:
        try:
            return self._cameras().get_config()[name]
        except KeyError:
            raise KeyError(name) from None

    def camera_pose(self, name: str):
        m = np.asarray(self._config(name)["cam2world_gl"], dtype=float)
        return m[:3, 3], m[:3, :3] @ _GL2OPTICAL

    def _camera(self, name: str):
        cams = self._cameras()
        if name in ("left_camera", "right_camera"):
            return getattr(cams, name)
        for camera, camera_name in zip(cams.static_camera_list, cams.static_camera_name):
            if camera_name == name:
                return camera
        raise KeyError(name)

    def camera_size(self, name: str, width: int, height: int):
        cam = self._camera(name)  # the head and wrist cameras take the config's size, the rest their own
        return int(cam.get_width()), int(cam.get_height())

    def render(self, name: str, width: int, height: int, depth: bool = False):
        """One camera, the way RoboTwin's Camera set reads it: colour from
        the Color texture, depth as -z of the Position texture (metres)."""
        task, cams = self._task, self._cameras()
        cams.update_wrist_camera(task.robot.left_camera.get_pose(), task.robot.right_camera.get_pose())
        task.scene.update_render()
        cam = self._camera(name)
        cam.take_picture()
        rgba = cam.get_picture("Color")
        rgb = (rgba[:, :, :3] * 255).clip(0, 255).astype(np.uint8)
        if not depth:
            return rgb
        position = cam.get_picture("Position")
        d = (-position[..., 2] * (rgba[:, :, 3] > 0)).astype(np.float32)
        return rgb, d

    def intrinsics(self, name: str, width: int, height: int) -> np.ndarray:
        return np.asarray(self._config(name)["intrinsic_cv"], dtype=float)

    # ------------------------------------------------------------ actuation
    def control_dt(self) -> float:
        return self._env.control_dt

    def mobile(self) -> bool:
        return False  # the base is in the articulation but RoboTwin never drives it

    def bind_arms(self, arms: list[tuple[np.ndarray, str]]) -> None:
        tags = ("left", "right")
        self._arms = [_Arm(tags[i], qadrs, hand_body) for i, (qadrs, hand_body) in enumerate(arms)]

    def has_gripper(self, arm: int) -> bool:
        return True

    def ee_wrench(self, arm: int):
        return np.zeros(3), np.zeros(3)

    def apply_tuning(self) -> None:
        pass

    def action(self, arm: int, dq: np.ndarray, grips: list[float],
               base_vel: np.ndarray | None) -> dict:
        """Per-arm absolute joint targets and gripper openness (bridge -1
        open / +1 close -> RoboTwin 1 open / 0 closed); other arms hold."""
        qpos = self.qpos()
        return {a.tag: (qpos[a.qadrs] + (dq if i == arm else 0.0), 0.0 if grips[i] > 0 else 1.0)
                for i, a in enumerate(self._arms)}

    def step(self, action) -> None:
        self._env.step(action)

    def fk(self, arm: int, q: np.ndarray | None = None):
        a = self._arms[arm]
        entity = self._entity
        if self._pinocchio[0] is not id(self._task):
            self._pinocchio = (id(self._task), entity.create_pinocchio_model())
            for x in self._arms:
                x.hand_index = None
        if a.hand_index is None:
            a.hand_index = [l.get_name() for l in entity.get_links()].index(a.hand_body)
        full = self.qpos()
        if q is not None:
            full[a.qadrs] = q
        model = self._pinocchio[1]
        model.compute_forward_kinematics(full)
        pose = entity.get_root_pose() * model.get_link_pose(a.hand_index)
        return (np.asarray(pose.p, dtype=float), _quat_to_mat(np.asarray(pose.q, dtype=float)))


ENGINE = RoboTwin
