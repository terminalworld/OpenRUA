"""robosuite (MuJoCo) engine: the bridge's view of a robosuite env.

The env is robosuite's own (or a benchmark wrapper around it, unwrapped
here through ``.env``); ``env.sim`` is its MjSim. Joint addressing,
poses and cameras read the MjSim model and data; actions follow
robosuite's action vector (per-robot slices of arm targets and a
gripper column, or the composite robot's ``create_action_vector``);
forward kinematics runs on a scratch MjData so the world never moves
for a jacobian.

Engine facts the ROS side used to hold and now only asks for:
mujoco cameras look along -Z and render bottom-up; the arm controller
moved from ``robot.controller`` (robosuite <= 1.4) to
``robot.part_controllers`` (1.5); ``ee_force`` is keyed by arm name
from 1.5 on; capbench's ``panda_joint_ctrl`` takes absolute joint
targets while the LIBERO forks take clipped deltas.
"""

from __future__ import annotations

import numpy as np

# mujoco cameras look along -Z; ROS optical frames look along +Z
# (REP 103/104): rotate pi about X to convert.
_MJ2OPTICAL = np.diag([1.0, -1.0, -1.0])


def arm_targets(q_current: list[np.ndarray], dq: np.ndarray, idx: int,
                absolute: bool, out_max: list[np.ndarray]) -> list[np.ndarray]:
    """Per-arm action columns when arm ``idx`` receives delta ``dq``.

    Every other arm HOLDS: absolute controllers get their current joint
    positions back, delta controllers get zeros. The commanded arm gets
    ``q + dq`` (absolute) or ``clip(dq / out_max)`` (delta), the same
    conversion the single-arm bridge always did at this boundary.
    """
    out = []
    for i, q in enumerate(q_current):
        if i == idx:
            out.append(q + dq if absolute
                       else np.clip(dq / out_max[i], -1.0, 1.0))
        else:
            out.append(q.copy() if absolute else np.zeros(len(q)))
    return out


class _Arm:
    """One arm's controller facts, bound to its qpos addresses."""

    def __init__(self, robot, raw, qadrs: np.ndarray, hand_body: str,
                 kp_scale: float):
        self.robot = robot
        self.qadrs = np.asarray(qadrs)
        self.hand_body = hand_body
        self.ctrl = self._controller()
        om = np.asarray(self.ctrl.output_max, dtype=float).flatten()
        self.out_max = np.where(om == 0, 1.0, om)
        # capbench's panda_joint_ctrl runs input_type "absolute": the
        # action IS the absolute joint target (radians, no clipping).
        # The bridge stays delta-shaped internally; the conversion
        # happens at the action-assembly boundary.
        self.absolute = getattr(self.ctrl, "input_type", "delta") == "absolute"
        self.kp_target = np.asarray(self.ctrl.kp, dtype=float).copy() * kp_scale
        self.hand_id = None  # lazy (needs the mujoco model)
        # Some robots mount a 0-DoF tool (no gripper): the action is
        # sized from the robot's own action dim, never assumed from the
        # config (a blind gripper column trips robosuite's action-dim
        # assert on every step).
        rdim = getattr(robot, "action_dim", None)
        if rdim is None:  # robosuite <= 1.4: single robot owns the env dim
            rdim = int(getattr(raw, "action_dim", len(self.qadrs) + 1))
        self.has_gripper = int(rdim) > len(self.qadrs)

    def _controller(self):
        """The arm's joint controller across the robosuite API break:
        <=1.4 exposes ``robot.controller``; 1.5 moved to composite
        controllers; the arm lives in ``robot.part_controllers`` under
        the first arm key (per-robot single-arm assemblies: "right")."""
        if hasattr(self.robot, "controller"):  # robosuite <= 1.4
            return self.robot.controller
        parts = self.robot.part_controllers  # robosuite >= 1.5
        for key in ("right", "left"):
            if key in parts:
                return parts[key]
        raise AttributeError(f"no arm controller among parts {list(parts)}")


class Robosuite:
    def __init__(self, env, cfg: dict):
        self._env = env
        self._raw = env.env if hasattr(env, "env") else env  # robosuite env
        # Joint-impedance stiffness tuned for 20Hz carrot streaming: the
        # stock kp (50) tracks only ~15% of a per-tick delta; measured
        # gain curve picks scale 10 (~0.92). Stored as an ABSOLUTE target
        # and re-applied after every reset (reset rebuilds the controller
        # and reverts kp); idempotent by construction.
        self._kp_scale = float(cfg.get("machine", {}).get("controller_kp_scale", 10.0))
        self._arms: list[_Arm] = []
        # Mobile-base composite: actions are assembled per part via the
        # robot's own create_action_vector; base velocities apply for
        # exactly the tick that carries them.
        parts = getattr(self._raw.robots[0], "part_controllers", None)
        self._mobile = bool(parts) and "base" in parts
        self._scratch = None  # FK scratch MjData (lazy)

    @property
    def _sim(self):
        # Read through the env every time: a hard reset replaces the MjSim.
        return self._raw.sim

    # ---------------------------------------------------------------- world
    def joints(self) -> list[tuple[str, int, int]]:
        model = self._sim.model
        out = []
        for name in model.joint_names:
            jid = model.joint_name2id(name)
            out.append((name, model.jnt_qposadr[jid], model.jnt_dofadr[jid]))
        return out

    def time(self) -> float:
        return float(self._sim.data.time)

    def qpos(self) -> np.ndarray:
        return self._sim.data.qpos

    def qvel(self) -> np.ndarray:
        return self._sim.data.qvel

    def effort(self) -> np.ndarray:
        return self._sim.data.qfrc_actuator

    def body_pose(self, name: str):
        sim = self._sim
        try:
            bid = sim.model.body_name2id(name)
        except Exception as exc:  # noqa: BLE001; robosuite raises ValueError
            raise KeyError(name) from exc
        return (sim.data.body_xpos[bid], sim.data.body_xquat[bid],
                sim.data.body_xmat[bid].reshape(3, 3))

    def site_pos(self, name: str):
        sim = self._sim
        try:
            sid = sim.model.site_name2id(name)
        except Exception as exc:  # noqa: BLE001
            raise KeyError(name) from exc
        return sim.data.site_xpos[sid]

    def objects(self) -> dict:
        obs = (self._raw._get_observations()
               if hasattr(self._raw, "_get_observations") else {})
        return {
            k: [float(v) for v in np.asarray(val).flatten()]
            for k, val in obs.items()
            if k.endswith("_pos") or k.endswith("_quat")
        }

    # -------------------------------------------------------------- cameras
    def camera_names(self) -> list[str]:
        return list(self._sim.model.camera_names)

    def camera_pose(self, name: str):
        sim = self._sim
        try:
            cid = sim.model.camera_name2id(name)
        except Exception as exc:  # noqa: BLE001
            raise KeyError(name) from exc
        return sim.data.cam_xpos[cid], sim.data.cam_xmat[cid].reshape(3, 3) @ _MJ2OPTICAL

    def render(self, name: str, width: int, height: int, depth: bool = False):
        if not depth:
            return self._sim.render(width=width, height=height, camera_name=name)[::-1]
        from robosuite.utils.camera_utils import get_real_depth_map

        rgb, d = self._sim.render(width=width, height=height, camera_name=name,
                                  depth=True)
        return (np.flipud(rgb).copy(),
                np.flipud(get_real_depth_map(self._sim, d)).astype(np.float32))

    def intrinsics(self, name: str, width: int, height: int) -> np.ndarray:
        from robosuite.utils.camera_utils import get_camera_intrinsic_matrix

        return get_camera_intrinsic_matrix(self._sim, name, height, width)

    # ------------------------------------------------------------ actuation
    def control_dt(self) -> float:
        return float(self._raw.control_timestep)

    def mobile(self) -> bool:
        return self._mobile

    def bind_arms(self, arms: list[tuple[np.ndarray, str]]) -> None:
        """Arm i drives robot i of the env (``env.robots``): its qpos
        addresses and hand body come from the config's per-arm view."""
        self._arms = [_Arm(self._raw.robots[i], self._raw, qadrs, hand_body,
                           self._kp_scale)
                      for i, (qadrs, hand_body) in enumerate(arms)]
        self.apply_tuning()

    def has_gripper(self, arm: int) -> bool:
        return self._arms[arm].has_gripper

    def ee_wrench(self, arm: int):
        robot = self._raw.robots[arm]
        f = getattr(robot, "ee_force", np.zeros(3))
        t = getattr(robot, "ee_torque", np.zeros(3))
        if isinstance(f, dict):  # robosuite >= 1.5: keyed by arm name
            f = next(iter(f.values()))
        if isinstance(t, dict):
            t = next(iter(t.values()))
        return f, t

    def apply_tuning(self) -> None:
        for a in self._arms:
            a.ctrl.kp = a.kp_target.copy()

    def action(self, arm: int, dq: np.ndarray, grips: list[float],
               base_vel: np.ndarray | None) -> np.ndarray:
        """Full env action for a delta on ONE arm; other arms hold.
        ``grips`` is every arm's persistent gripper state (-1 open, +1
        close); ``base_vel`` is [vx, vy, wz] for a driven base."""
        qpos = self._sim.data.qpos
        a = self._arms[arm]
        if self._mobile:
            q = qpos[a.qadrs]
            av = (q + dq if a.absolute
                  else np.clip(dq / a.out_max, -1.0, 1.0))
            base_vel = np.zeros(3) if base_vel is None else base_vel
            moving = bool(np.any(base_vel))
            return self._raw.robots[0].create_action_vector({
                "right": av,
                "right_gripper": np.array([grips[arm]]),
                "base": base_vel,
                # base mode (+1) makes the arm track its goal while the
                # base moves; arm mode (-1) otherwise. Torso omitted -> 0.
                "base_mode": 1.0 if moving else -1.0,
            })
        if len(self._arms) == 1:
            grip = [grips[arm]] if a.has_gripper else []
            if a.absolute:  # absolute target = current + per-tick delta
                q = qpos[a.qadrs]
                return np.concatenate([q + dq, grip])
            return np.concatenate(
                [np.clip(dq / a.out_max, -1.0, 1.0), grip])
        qs = [qpos[x.qadrs] for x in self._arms]
        targets = arm_targets(qs, dq, arm, a.absolute,
                              [x.out_max for x in self._arms])
        slices = []
        for x, g, t in zip(self._arms, grips, targets):
            grip = [g] if x.has_gripper else []
            slices.append(np.concatenate([t, grip]))
        return np.concatenate(slices)

    def step(self, action: np.ndarray) -> None:
        self._env.step(action)

    def fk(self, arm: int, q: np.ndarray | None = None):
        """FK of the arm's hand body; q=None reads the live sim state."""
        import mujoco

        a = self._arms[arm]
        model = self._sim.model._model
        if self._scratch is None:
            self._scratch = mujoco.MjData(model)
        if a.hand_id is None:
            a.hand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, a.hand_body)
        d = self._scratch
        d.qpos[:] = self._sim.data.qpos
        if q is not None:
            d.qpos[a.qadrs] = q
        mujoco.mj_kinematics(model, d)
        return d.xpos[a.hand_id].copy(), d.xmat[a.hand_id].reshape(3, 3).copy()


ENGINE = Robosuite
