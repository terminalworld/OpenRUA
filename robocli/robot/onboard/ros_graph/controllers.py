"""Command side of the bridge: twist servo (L4) + FollowJointTrajectory
(L3) + GripperCommand; all above a JOINT_POSITION base action space.

Real-robot shape: on a Franka both graph ports sit above a joint-level
controller; FollowJointTrajectory tracks joint targets natively, and
moveit_servo turns twists into joint commands via differential IK. The
bridge mirrors exactly that. (OSC delta-EE was tried first and rejected
by measurement: its null-space bias fights joint-space goals; 0.23 rad
residual on a two-point FJT goal, dense points don't help.)

Multi-arm machines (capbench twoarm suites): one port set per entry of
``machine.arms`` (run.normalize_arms materialized it); commands address
one arm, every other arm holds its current configuration for the ticks
the command spans (arms.arm_targets owns that math). Robot i in the
simulator's ``env.robots`` is arm i; the full action is the per-robot
slices in that order.

Paused-clock semantics: **one incoming command message = one sim step**;
a trajectory/gripper goal advances the world by the sim time it takes.
Commands serialize through the sim thread in arrival order.
"""

from __future__ import annotations

import numpy as np
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionServer

from .arms import arm_specs, arm_targets
from .joints import build_joint_map

_FD_EPS = 1e-6  # finite-difference step for the jacobian


class _ArmUnit:
    """One arm's ports and state; the owner assembles full actions."""

    def __init__(self, owner: "CommandPorts", node, index: int, spec: dict):
        self.owner = owner
        self.index = index
        self.label = spec.get("label", "")
        self.robot = owner._raw.robots[index]
        self.ctrl = self._controller()
        om = np.asarray(self.ctrl.output_max, dtype=float).flatten()
        self.out_max = np.where(om == 0, 1.0, om)
        # capbench's panda_joint_ctrl runs input_type "absolute": the
        # action IS the absolute joint target (radians, no clipping).
        # The bridge stays delta-shaped internally; the conversion
        # happens at the action-assembly boundary.
        self.absolute = getattr(self.ctrl, "input_type", "delta") == "absolute"
        self.kp_target = (np.asarray(self.ctrl.kp, dtype=float).copy()
                          * owner._kp_scale)
        self.grip = -1.0  # persistent gripper state: -1 open, +1 close
        joints = spec.get("joints") or [f"panda_joint{i}" for i in range(1, 8)]
        self.arm_names = joints
        self.qadrs = np.array([owner._jmap[n][0] for n in joints])
        self.hand_body = spec.get("hand_body", f"robot{index}_right_hand")
        self._hand_id = None  # lazy (needs mujoco model)
        # Wipe-class assemblies mount a 0-DoF tool (no gripper): sizing
        # from the robot's own action dim, never assumed from the config
        # (2026-08-11 wipe canary: a blind gripper column tripped
        # robosuite's action-dim assert on every step).
        rdim = getattr(self.robot, "action_dim", None)
        if rdim is None:  # robosuite <= 1.4: single robot owns the env dim
            rdim = int(getattr(owner._raw, "action_dim", len(joints) + 1))
        self.has_gripper = int(rdim) > len(joints)

        ports = spec.get("ports", {})
        if ports.get("twist"):
            node.create_subscription(
                TwistStamped, ports["twist"],
                lambda msg, a=self: owner._on_twist(a, msg), 10)
        # GripperCommand action (franka_ros2 name). position >= threshold
        # means open (Franka: 0.0 closed .. 0.04/finger open); the goal
        # runs settle_steps control ticks so the fingers physically move.
        if ports.get("gripper") and self.has_gripper:
            ActionServer(node, GripperCommand, ports["gripper"],
                         lambda gh, a=self: owner._on_gripper_goal(a, gh))
        # FollowJointTrajectory (L3): native joint-space tracking.
        if ports.get("trajectory"):
            ActionServer(node, FollowJointTrajectory, ports["trajectory"],
                         lambda gh, a=self: owner._on_traj_goal(a, gh))

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


class CommandPorts:
    def __init__(self, node, env, cfg: dict, on_step, sim=None):
        self._env = env
        self._on_step = on_step  # callback: publish sensors after stepping
        self._runner = sim  # SimJobRunner: all env access goes through it

        raw = env.env if hasattr(env, "env") else env
        self._raw = raw
        self._control_dt = float(raw.control_timestep)
        machine = cfg.get("machine", {})
        # Joint-impedance stiffness tuned for 20Hz carrot streaming: the
        # stock kp (50) tracks only ~15% of a per-tick delta; measured
        # gain curve picks scale 10 (~0.92). Stored as an ABSOLUTE target
        # and re-applied after every reset (reset rebuilds the controller
        # and reverts kp); idempotent by construction.
        self._kp_scale = float(machine.get("controller_kp_scale", 10.0))
        # Arm joints in action order (published franka names -> qpos/dof).
        self._jmap = {n: (q, d) for n, q, d in build_joint_map(env, cfg)}

        self._arms = [_ArmUnit(self, node, i, spec)
                      for i, spec in enumerate(arm_specs(machine))]
        self._multi = len(self._arms) > 1
        self.apply_tuning()

        # Mobile-base composite (robocasa leg): actions are assembled per
        # part via the robot's own create_action_vector; base velocities
        # apply for exactly the tick that carries them (paused-clock
        # analog of a real base's command watchdog). Single-arm only.
        robot0 = raw.robots[0]
        parts = getattr(robot0, "part_controllers", None)
        self._mobile = bool(parts) and "base" in parts
        if self._mobile and self._multi:
            raise ValueError("mobile base + multiple arms is not a "
                             "supported assembly")
        self._base_vel = np.zeros(3)  # vx, vy, wz (JOINT_VELOCITY base)
        # Base command policy (design-decisions 2026-08-12):
        #   "queue"  (mainline): one message = one tick, deterministic;
        #            bounded by _BASE_QUEUE_MAX as a flood fuse; the
        #            2026-08-12 composite canaries flooded thousands of
        #            ticks of "displacement debt" and crushed the sim.
        #   "latest" (real-robot semantics): a new message REPLACES the
        #            pending setpoint (velocity knob, not a queued debt);
        #            reserved for the free-clock/real-robot legs.
        import threading as _threading

        self._base_policy = str(
            cfg.get("protocol", {}).get("base_cmd_policy", "queue"))
        self._base_lock = _threading.Lock()
        self._base_pending = 0
        self._base_dropped = 0
        self._latest_base = np.zeros(3)

        control = machine.get("control", {})
        self._grip_threshold = float(
            control.get("gripper", {}).get("open_threshold_m", 0.02))
        self._settle_steps = int(
            control.get("gripper", {}).get("settle_steps", 10))
        self._traj_tol_rad = float(
            control.get("trajectory", {}).get("goal_tolerance_rad", 0.05))
        self._traj_settle = int(
            control.get("trajectory", {}).get("settle_steps", 20))
        ports = machine.get("ports", {})  # base ports live at machine level
        if self._mobile and ports.get("base_twist"):
            from geometry_msgs.msg import Twist  # plain Twist: /cmd_vel

            node.create_subscription(
                Twist, ports["base_twist"], self._on_base_twist, 10)

    def apply_tuning(self) -> None:
        """(Re-)apply controller tuning; call after every env reset."""
        for a in self._arms:
            a.ctrl.kp = a.kp_target.copy()

    # -------------------------------------------------------------- stepping
    def _action(self, arm: _ArmUnit, dq: np.ndarray) -> np.ndarray:
        """Full env action for a delta on ONE arm; other arms hold."""
        sim = self._env.sim
        if self._mobile:
            q = sim.data.qpos[arm.qadrs]
            av = (q + dq if arm.absolute
                  else np.clip(dq / arm.out_max, -1.0, 1.0))
            moving = bool(np.any(self._base_vel))
            return self._raw.robots[0].create_action_vector({
                "right": av,
                "right_gripper": np.array([arm.grip]),
                "base": self._base_vel,
                # base mode (+1) makes the arm track its goal while the
                # base moves; arm mode (-1) otherwise. Torso omitted -> 0.
                "base_mode": 1.0 if moving else -1.0,
            })
        if not self._multi:
            grip = [arm.grip] if arm.has_gripper else []
            if arm.absolute:  # absolute target = current + per-tick delta
                q = sim.data.qpos[arm.qadrs]
                return np.concatenate([q + dq, grip])
            return np.concatenate(
                [np.clip(dq / arm.out_max, -1.0, 1.0), grip])
        qs = [sim.data.qpos[a.qadrs] for a in self._arms]
        targets = arm_targets(qs, dq, arm.index, arm.absolute,
                              [a.out_max for a in self._arms])
        slices = []
        for a, t in zip(self._arms, targets):
            grip = [a.grip] if a.has_gripper else []
            slices.append(np.concatenate([t, grip]))
        return np.concatenate(slices)

    def step(self, action: np.ndarray) -> None:
        """Advance the world by one control tick (sim-thread job)."""

        def job():
            self._env.step(action)
            self._on_step()

        self._runner.submit(job, wait=False)

    # ----------------------------------------------------------------- twist
    def _on_twist(self, arm: _ArmUnit, msg: TwistStamped) -> None:
        v6 = np.array(
            [
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z,
                msg.twist.angular.x,
                msg.twist.angular.y,
                msg.twist.angular.z,
            ],
            dtype=float,
        )

        def job():
            # Differential IK (moveit_servo semantics): dq = J^+ (v * dt).
            jac = self._jacobian(arm)
            dq, *_ = np.linalg.lstsq(jac, v6 * self._control_dt, rcond=None)
            self._env.step(self._action(arm, dq))
            self._on_step()

        self._runner.submit(job, wait=False)

    _BASE_QUEUE_MAX = 4  # flood fuse: max queued base ticks (policy "queue")

    def _on_base_twist(self, msg) -> None:
        """/cmd_vel (plain Twist): one message = one control tick of base
        motion at [vx, vy, wz]; the velocity does NOT persist beyond its
        tick. Messages queue up to _BASE_QUEUE_MAX; excess is dropped
        (logged), like a real base's lossy command transport. Under policy
        "latest" a new message replaces the pending setpoint instead."""
        v = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=float)

        with self._base_lock:
            self._latest_base = v
            if self._base_policy == "latest":
                if self._base_pending:
                    return  # the queued tick consumes the newest setpoint
            elif self._base_pending >= self._BASE_QUEUE_MAX:
                self._base_dropped += 1
                if self._base_dropped % 100 == 1:
                    import sys
                    print(f"[base] queue full, dropped "
                          f"{self._base_dropped} cmd_vel msgs total",
                          file=sys.stderr, flush=True)
                return
            self._base_pending += 1

        arm = self._arms[0]

        def job():
            with self._base_lock:
                self._base_pending -= 1
                self._base_vel = (self._latest_base
                                  if self._base_policy == "latest" else v)
            try:
                self._env.step(self._action(arm, np.zeros(len(arm.qadrs))))
                self._on_step()
            finally:
                self._base_vel = np.zeros(3)

        self._runner.submit(job, wait=False)

    def _hand_pose(self, arm: _ArmUnit, q: np.ndarray | None = None):
        """FK of the arm's hand body; q=None reads the live sim state."""
        import mujoco

        sim = self._env.sim
        model = sim.model._model
        if not hasattr(self, "_scratch"):
            self._scratch = mujoco.MjData(model)
        if arm._hand_id is None:
            arm._hand_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, arm.hand_body)
        d = self._scratch
        d.qpos[:] = sim.data.qpos
        if q is not None:
            d.qpos[arm.qadrs] = q
        mujoco.mj_kinematics(model, d)
        return (d.xpos[arm._hand_id].copy(),
                d.xmat[arm._hand_id].reshape(3, 3).copy())

    def _jacobian(self, arm: _ArmUnit) -> np.ndarray:
        """6x7 hand jacobian by finite differences on FK (deterministic,
        no raw-binding dependency)."""
        sim = self._env.sim
        q0 = sim.data.qpos[arm.qadrs].copy()
        p0, r0 = self._hand_pose(arm, q0)
        jac = np.zeros((6, len(q0)))
        for i in range(len(q0)):
            q = q0.copy()
            q[i] += _FD_EPS
            p, r = self._hand_pose(arm, q)
            jac[:3, i] = (p - p0) / _FD_EPS
            r_err = r @ r0.T
            jac[3:, i] = (
                np.array(
                    [
                        r_err[2, 1] - r_err[1, 2],
                        r_err[0, 2] - r_err[2, 0],
                        r_err[1, 0] - r_err[0, 1],
                    ]
                )
                / (2 * _FD_EPS)
            )
        return jac

    # --------------------------------------------------------------- gripper
    def set_gripper(self, arm: _ArmUnit, close: bool) -> None:
        if not arm.has_gripper:
            return
        arm.grip = 1.0 if close else -1.0
        # A gripper command is also a command: it advances the world.
        def job():
            for _ in range(self._settle_steps):
                self._env.step(self._action(arm, np.zeros(len(arm.qadrs))))
            self._on_step()

        self._runner.submit(job, wait=True)

    def _on_gripper_goal(self, arm: _ArmUnit, goal_handle):
        close = goal_handle.request.command.position < self._grip_threshold
        self.set_gripper(arm, close)
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.reached_goal = True
        result.stalled = False
        return result

    # ------------------------------------------------------------ trajectory
    def _on_traj_goal(self, arm: _ArmUnit, goal_handle):
        traj = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()
        try:
            err = self._runner.submit(
                lambda: self._exec_traj(arm, traj), wait=True)
        except Exception as exc:  # noqa: BLE001
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"{type(exc).__name__}: {exc}"
            return result
        if err <= self._traj_tol_rad:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        else:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        result.error_string = f"final max joint error {err:.4f} rad"
        return result

    def _exec_traj(self, arm: _ArmUnit, traj) -> float:
        """Run on the sim thread. Returns the final max joint error (rad)."""
        import sys

        sim = self._env.sim
        print(
            f"[fjt{':' + arm.label if arm.label else ''}] "
            f"names={list(traj.joint_names)} pts={len(traj.points)}",
            file=sys.stderr, flush=True,
        )
        foreign = [n for n in traj.joint_names if n not in arm.arm_names]
        if foreign:
            raise ValueError(
                f"joints {foreign} are not on this arm; this port drives "
                f"{arm.arm_names}")
        qadrs = np.array([self._jmap[n][0] for n in traj.joint_names])
        pts = sorted(
            traj.points,
            key=lambda p: p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
        )
        times = np.array(
            [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in pts]
        )
        qs = np.array([p.positions for p in pts])
        if times[0] > 0:  # implicit start at current configuration
            times = np.concatenate([[0.0], times])
            qs = np.vstack([sim.data.qpos[qadrs], qs])

        # Column order of the action = the arm's qadrs order; map goal joints.
        order = [int(np.where(arm.qadrs == a)[0][0]) for a in qadrs]

        def tick(q_target: np.ndarray) -> None:
            dq = np.zeros(len(arm.qadrs))
            dq[order] = q_target - sim.data.qpos[qadrs]
            self._env.step(self._action(arm, dq))

        t = self._control_dt
        end = float(times[-1])
        while t <= end:
            i = int(np.searchsorted(times, t, side="right")) - 1
            i = min(i, len(times) - 2)
            # clamp: duplicate timestamps (MoveIt emits them) would blow the
            # interpolation factor up through the 1e-9 guard otherwise
            a = min(1.0, max(0.0, (t - times[i]) / max(times[i + 1] - times[i], 1e-9)))
            tick(qs[i] + a * (qs[i + 1] - qs[i]))
            t += self._control_dt
        for _ in range(self._traj_settle):  # converge on the final point
            tick(qs[-1])
        self._on_step()
        err_vec = sim.data.qpos[qadrs] - qs[-1]
        print(
            f"[fjt] times[0..2]={times[:3]} times[-3:]={times[-3:]}\n"
            f"[fjt] goal={np.round(qs[-1], 3)}\n"
            f"[fjt] final={np.round(sim.data.qpos[qadrs], 3)}\n"
            f"[fjt] err={np.round(err_vec, 3)}",
            file=sys.stderr, flush=True,
        )
        return float(np.max(np.abs(err_vec)))
