"""ManiSkill 3 (SAPIEN, PhysX CPU) engine: the bridge's view of a ManiSkill env.

The env is gymnasium's wrapper around a ManiSkill ``BaseEnv``
(``env.unwrapped``); the robot is its agent's articulation, cameras are
its sensors and human render cameras. One sub-environment
(``num_envs=1``, ``sim_backend="physx_cpu"``): every batched tensor is
read at index 0. Rendering goes through Vulkan; with no GPU SAPIEN
finds lavapipe.

Engine facts kept here: SAPIEN poses are (p, q) with q as w x y z;
``cam2world_gl`` is the OpenGL convention (a camera looks along -Z), so
the optical frame is a rotation of pi about X; sensor depth arrives in
millimetres as int16; the ``pd_joint_pos`` arm controller takes
absolute joint targets in radians while the gripper's
``pd_joint_pos`` (mimic) is normalised, +1 open and -1 closed; the
pinocchio model gives link poses in the robot's root frame.
"""

from __future__ import annotations

import numpy as np

from .frames import GL2OPTICAL, quat_to_mat


def _np(t) -> np.ndarray:
    """A torch tensor (batched, index 0) or array -> a float64 numpy array."""
    if hasattr(t, "detach"):
        t = t.detach().cpu().numpy()
    a = np.asarray(t)
    return a[0] if a.ndim >= 1 and a.shape[0] == 1 else a  # drop the batch axis


class _Arm:
    def __init__(self, agent, qadrs: np.ndarray, hand_body: str, joint_names: list[str]):
        self.agent = agent
        self.qadrs = np.asarray(qadrs)
        self.hand_body = hand_body
        ctrls = agent.controller.controllers  # CombinedController: arm, gripper
        self.arm_ctrl = ctrls["arm"]
        self.grip_ctrl = ctrls.get("gripper")
        self.has_gripper = self.grip_ctrl is not None
        # Action layout: the combined controller concatenates its parts in
        # dict order; the arm's part lists its joints in its own order.
        offset, self.arm_slots, self.grip_slots = 0, {}, []
        for name, c in ctrls.items():
            dim = int(np.prod(c.action_space.shape))
            if name == "arm":
                for k, jn in enumerate(c.config.joint_names):
                    self.arm_slots[jn] = offset + k
            elif name == "gripper":
                self.grip_slots = list(range(offset, offset + dim))
            offset += dim
        self.action_dim = offset
        # qpos index -> action slot, in qadrs order.
        self.slot_of = [self.arm_slots[joint_names[i]] for i in self.qadrs]
        self.hand_index = None
        self.pinocchio = None


class ManiSkill:
    def __init__(self, env, cfg: dict):
        self._env = env
        self._u = env.unwrapped
        self._arms: list[_Arm] = []
        self._joint_names = [j.name for j in self._robot(0).get_active_joints()]

    def _robot(self, i: int):
        return self._agent(i).robot

    def _agent(self, i: int):
        agents = getattr(self._u.agent, "agents", None)  # MultiAgent
        return agents[i] if agents is not None else self._u.agent

    # ---------------------------------------------------------------- world
    def joints(self) -> list[tuple[str, int, int]]:
        return [(n, i, i) for i, n in enumerate(self._joint_names)]

    def time(self) -> float:
        return float(_np(self._u.elapsed_steps)) * self.control_dt()

    def qpos(self) -> np.ndarray:
        return _np(self._robot(0).get_qpos()).astype(float)

    def qvel(self) -> np.ndarray:
        return _np(self._robot(0).get_qvel()).astype(float)

    def effort(self) -> np.ndarray:
        return _np(self._robot(0).get_qf()).astype(float)

    def _pose_of(self, name: str):
        for i in range(len(self._arms) or 1):
            link = self._robot(i).find_link_by_name(name)
            if link is not None:
                return link.pose.sp
        actor = self._u.scene.actors.get(name)
        if actor is not None:
            return actor.pose.sp
        raise KeyError(name)

    def body_pose(self, name: str):
        pose = self._pose_of(name)
        p, q = np.asarray(pose.p, dtype=float), np.asarray(pose.q, dtype=float)
        return p, q, quat_to_mat(q)

    def site_pos(self, name: str):
        raise KeyError(name)  # SAPIEN has no sites

    def objects(self) -> dict:
        out = {}
        for name, actor in self._u.scene.actors.items():
            pose = actor.pose.sp
            out[f"{name}_pos"] = [float(v) for v in pose.p]
            out[f"{name}_quat"] = [float(v) for v in pose.q]
        return out

    # -------------------------------------------------------------- cameras
    def _cameras(self) -> dict:
        cams = dict(self._u._sensors)
        cams.update(getattr(self._u, "_human_render_cameras", {}))
        return cams

    def camera_names(self) -> list[str]:
        return list(self._cameras())

    def _camera(self, name: str):
        try:
            return self._cameras()[name]
        except KeyError:
            raise KeyError(name) from None

    def camera_pose(self, name: str):
        m = _np(self._camera(name).get_params()["cam2world_gl"])
        return m[:3, 3].astype(float), m[:3, :3].astype(float) @ GL2OPTICAL

    def camera_size(self, name: str, width: int, height: int):
        cam = self._camera(name)  # sized at creation (the loader passes the config's size)
        return int(cam.config.width), int(cam.config.height)

    def render(self, name: str, width: int, height: int, depth: bool = False):
        cam = self._camera(name)
        self._u.scene.update_render()
        cam.capture()
        obs = cam.get_obs(rgb=True, depth=depth, position=False, segmentation=False)
        rgb = _np(obs["rgb"]).astype(np.uint8)
        if not depth:
            return rgb
        d = _np(obs["depth"])
        return rgb, (d[..., 0].astype(np.float32) / 1000.0)  # mm -> m

    def intrinsics(self, name: str, width: int, height: int) -> np.ndarray:
        return _np(self._camera(name).get_params()["intrinsic_cv"]).astype(float)

    # ------------------------------------------------------------ actuation
    def control_dt(self) -> float:
        return float(self._u.control_timestep)

    def mobile(self) -> bool:
        return False

    def bind_arms(self, arms: list[tuple[np.ndarray, str]]) -> None:
        self._arms = [_Arm(self._agent(i), qadrs, hand_body, self._joint_names)
                      for i, (qadrs, hand_body) in enumerate(arms)]

    def has_gripper(self, arm: int) -> bool:
        return self._arms[arm].has_gripper

    def ee_wrench(self, arm: int):
        return np.zeros(3), np.zeros(3)  # no force sensing on the SAPIEN arm

    def apply_tuning(self) -> None:
        pass  # gains are the controller config's; a reset keeps them

    def action(self, arm: int, dq: np.ndarray, grips: list[float],
               base_vel: np.ndarray | None) -> np.ndarray:
        """Absolute joint targets: the commanded arm at q + dq, every other
        arm at its current q, grippers at their persistent state (bridge
        -1 open / +1 close -> ManiSkill +1 open / -1 close)."""
        qpos = self.qpos()
        parts = []
        for i, a in enumerate(self._arms):
            act = np.zeros(a.action_dim)
            q = qpos[a.qadrs] + (dq if i == arm else 0.0)
            act[a.slot_of] = q
            if a.has_gripper:
                act[a.grip_slots] = -float(grips[i])
            parts.append(act)
        if len(parts) == 1:
            return parts[0]
        return {self._agent(i).uid: p for i, p in enumerate(parts)}

    def step(self, action) -> None:
        self._env.step(action)

    def fk(self, arm: int, q: np.ndarray | None = None):
        a = self._arms[arm]
        robot = self._robot(arm)
        if a.pinocchio is None:
            a.pinocchio = robot.create_pinocchio_model()
            a.hand_index = [l.name for l in robot.get_links()].index(a.hand_body)
        full = self.qpos()
        if q is not None:
            full[a.qadrs] = q
        a.pinocchio.compute_forward_kinematics(full)
        # Link poses come in the robot's root frame; the root sits at robot.pose.
        pose = robot.pose.sp * a.pinocchio.get_link_pose(a.hand_index)
        return (np.asarray(pose.p, dtype=float),
                quat_to_mat(np.asarray(pose.q, dtype=float)))


ENGINE = ManiSkill
