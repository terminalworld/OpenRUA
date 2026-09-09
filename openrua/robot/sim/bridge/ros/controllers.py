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
the command spans. The engine assembles the action (``engine.action``)
and owns every controller fact; this side holds the ports, the joint
naming and the gripper state, and does the trajectory and servo math.

Paused-clock semantics: **one incoming command message = one sim step**;
a trajectory/gripper goal advances the world by the sim time it takes.
Commands serialize through the sim thread in arrival order.
"""

from __future__ import annotations

import numpy as np
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionServer

from .arms import arm_specs
from .joints import build_joint_map

_FD_EPS = 1e-6  # finite-difference step for the jacobian


class _ArmUnit:
    """One arm's ports and state; the owner assembles full actions."""

    def __init__(self, owner: "CommandPorts", index: int, spec: dict):
        self.owner = owner
        self.index = index
        self.label = spec.get("label", "")
        self.grip = -1.0  # persistent gripper state: -1 open, +1 close
        joints = spec.get("joints") or [f"panda_joint{i}" for i in range(1, 8)]
        self.arm_names = joints
        self.qadrs = np.array([owner._jmap[n][0] for n in joints])
        self.hand_body = spec.get("hand_body", f"robot{index}_right_hand")
        self.has_gripper = False  # the engine answers once the arms are bound
        self.ports = spec.get("ports", {})

    def open_ports(self, node) -> None:
        owner, ports = self.owner, self.ports
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


class CommandPorts:
    def __init__(self, node, engine, cfg: dict, on_step, sim=None):
        self._engine = engine
        self._on_step = on_step  # callback: publish sensors after stepping
        self._runner = sim  # Worker: all env access goes through it

        self._control_dt = engine.control_dt()
        machine = cfg.get("machine", {})
        # Arm joints in action order (published franka names -> qpos/dof).
        self._jmap = {n: (q, d) for n, q, d in build_joint_map(engine, cfg)}

        self._arms = [_ArmUnit(self, i, spec)
                      for i, spec in enumerate(arm_specs(machine))]
        self._multi = len(self._arms) > 1
        # The engine learns which joints and hand each arm drives, and
        # answers with the controller facts (gripper or not).
        engine.bind_arms([(a.qadrs, a.hand_body) for a in self._arms])
        for a in self._arms:
            a.has_gripper = engine.has_gripper(a.index)
            a.open_ports(node)

        # Mobile base: base velocities apply for exactly the tick that
        # carries them (paused-clock analog of a real base's command
        # watchdog). Single-arm only.
        self._mobile = engine.mobile()
        if self._mobile and self._multi:
            raise ValueError("mobile base + multiple arms is not a "
                             "supported combination")
        self._base_vel = np.zeros(3)  # vx, vy, wz (JOINT_VELOCITY base)
        # Base command policy:
        #   "queue"  (default): one message = one tick, deterministic;
        #            bounded by _BASE_QUEUE_MAX so a flood of messages
        #            cannot queue thousands of ticks of displacement.
        #   "latest" (real-robot semantics): a new message replaces the
        #            pending setpoint (a velocity setting, not a queued
        #            debt); for free-running clocks.
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
        self._engine.apply_tuning()

    # -------------------------------------------------------------- stepping
    def _tick(self, arm: _ArmUnit, dq: np.ndarray) -> None:
        """One control tick: a delta on ONE arm, other arms hold, every
        gripper at its persistent state, the base at this tick's velocity."""
        self._engine.step(self._engine.action(
            arm.index, dq, [a.grip for a in self._arms],
            self._base_vel if self._mobile else None))

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
            self._tick(arm, dq)
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
                self._tick(arm, np.zeros(len(arm.qadrs)))
                self._on_step()
            finally:
                self._base_vel = np.zeros(3)

        self._runner.submit(job, wait=False)

    def _jacobian(self, arm: _ArmUnit) -> np.ndarray:
        """6x7 hand jacobian by finite differences on the engine's FK
        (deterministic, no raw-binding dependency)."""
        fk = self._engine.fk
        q0 = self._engine.qpos()[arm.qadrs].copy()
        p0, r0 = fk(arm.index, q0)
        jac = np.zeros((6, len(q0)))
        for i in range(len(q0)):
            q = q0.copy()
            q[i] += _FD_EPS
            p, r = fk(arm.index, q)
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
                self._tick(arm, np.zeros(len(arm.qadrs)))
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

        qpos = self._engine.qpos  # read anew before every use: an engine may hand out copies
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
            qs = np.vstack([qpos()[qadrs], qs])

        # Column order of the action = the arm's qadrs order; map goal joints.
        order = [int(np.where(arm.qadrs == a)[0][0]) for a in qadrs]

        def tick(q_target: np.ndarray) -> None:
            dq = np.zeros(len(arm.qadrs))
            dq[order] = q_target - qpos()[qadrs]
            self._tick(arm, dq)

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
        err_vec = qpos()[qadrs] - qs[-1]
        print(
            f"[fjt] times[0..2]={times[:3]} times[-3:]={times[-3:]}\n"
            f"[fjt] goal={np.round(qs[-1], 3)}\n"
            f"[fjt] final={np.round(qpos()[qadrs], 3)}\n"
            f"[fjt] err={np.round(err_vec, 3)}",
            file=sys.stderr, flush=True,
        )
        return float(np.max(np.abs(err_vec)))
