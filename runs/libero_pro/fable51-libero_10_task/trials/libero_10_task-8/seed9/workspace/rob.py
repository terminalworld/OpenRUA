#!/usr/bin/env python3
"""Small helper library for this Panda: joint read, FK, IK, trajectory,
gripper, servo. Poses are in the ARM BASE frame (panda_link0) unless
leave frame_id empty. IK ignores yaw about hand z -> fix joint7 afterwards.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
WORLD_TO_BASE = np.array([-0.66, 0.0, 0.912])  # position of panda_link0 in world


def w2b(p):
    return np.asarray(p, float) - WORLD_TO_BASE


def b2w(p):
    return np.asarray(p, float) + WORLD_TO_BASE


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quat_about_z(theta):
    return np.array([0.0, 0.0, math.sin(theta / 2), math.cos(theta / 2)])


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self._clk = None
        self.node.create_subscription(Clock, "/clock", self._on_clk, 10)
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 10)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self._f0 = np.zeros(3)  # wrench baseline for servo_to max_force

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def _on_clk(self, m):
        self._clk = m.clock.sec + m.clock.nanosec * 1e-9

    def settle(self, quiet=1.5):
        """spin until the (paused) sim clock has stopped changing for `quiet`
        wall seconds, i.e. the bridge has drained all queued commands"""
        last = time.time()
        c = self._clk
        while time.time() - last < quiet:
            self.spin(0.2)
            if self._clk != c:
                c = self._clk
                last = time.time()
        return c

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---- sensing ----
    def joints(self, fresh=True, wait=3.0):
        """latest joint state; with fresh=True wait up to `wait` s for a new
        message (sim may be paused -> fall back to the last cached one)"""
        last = self._js
        if fresh:
            self._js = None
        end = time.time() + (wait if last is not None else 15)
        while self._js is None and time.time() < end:
            self.spin(0.1)
        if self._js is None:
            if last is None:
                raise RuntimeError("no /joint_states")
            self._js = last
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 5
        while self._wr is None and time.time() < end:
            self.spin(0.1)
        if self._wr is None:
            return None
        f = self._wr.wrench.force
        return np.array([f.x, f.y, f.z])

    def fk(self, q=None, link="panda_hand"):
        """hand pose in base frame: (pos[3], quat[4] xyzw)"""
        if q is None:
            q = self.arm_q()
        self.fk_cli.wait_for_service(5)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed: {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w]))

    def tcp(self, q=None):
        """fingertip-centre position in base frame"""
        p, quat = self.fk(q)
        return p + TCP * quat_to_R(quat)[:, 2], quat

    def ik(self, pos, quat, seed=None, at_tcp=False, timeout=10.0):
        """pos/quat in base frame -> arm joint array, or None"""
        pos = np.asarray(pos, float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(quat)[:, 2]
        if seed is None:
            seed = self.arm_q()
        self.ik_cli.wait_for_service(5)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        (p.orientation.x, p.orientation.y,
         p.orientation.z, p.orientation.w) = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.timeout = Duration(sec=int(timeout),
                                          nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"IK failed code={r and r.error_code.val}", file=sys.stderr)
            return None
        d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return np.array([d[j] for j in ARM])

    # ---- action ----
    def move(self, q, seconds=3.0, waypoints=None):
        """one FJT goal; waypoints = list of (q, t) before the final q"""
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        for wq, wt in (waypoints or []):
            pt = JointTrajectoryPoint(positions=[float(v) for v in wq])
            pt.time_from_start = Duration(sec=int(wt), nanosec=int((wt % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(self.arm_q() - np.asarray(q)).max()
        print(f"move done code={code} max_joint_err={err:.4f}")
        return code, err

    def gripper(self, width):
        self.grip.wait_for_server(10)
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        print(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} "
              f"gap={self.finger_gap():.4f}")
        return r

    def servo(self, lin, ang=(0, 0, 0), ticks=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            self.spin(dt)

    def burst(self, lin, ang=(0, 0, 0), n=20, dt=0.08, zeros=10):
        """n twist msgs (each = 0.05 s sim) at a rate the bridge keeps up
        with, then wait for the sim to drain so sensor reads are exact"""
        self.servo(lin, ang, ticks=n, dt=dt)
        self.servo((0, 0, 0), ticks=zeros, dt=dt)  # flush servo velocity
        self.settle()

    def servo_to(self, target, speed=0.05, tol=0.001, n=20, max_bursts=40,
                 watch_gap=None, max_force=None, gain=1.0):
        """closed-loop straight-line TCP move (world frame): bursts with
        exact state read in between. gain = m of motion per (m/s * s) cmd"""
        target = np.asarray(target, float)
        for i in range(max_bursts):
            t, _ = self.tcp()
            if max_force is not None:
                f = self.wrench()
                if f is not None and np.linalg.norm(f - self._f0) > max_force:
                    print(f"servo_to: force {np.round(f,2)} exceeds limit, stopping at {t.round(4)}")
                    return False
            err = target - t
            dist = np.linalg.norm(err)
            if dist < tol:
                print(f"servo_to reached {t.round(4)} (err {dist*1000:.1f} mm)")
                return True
            if watch_gap is not None:
                g = self.finger_gap()
                if abs(g - watch_gap) > 0.004:
                    print(f"servo_to: gap changed {watch_gap:.4f}->{g:.4f}, stopping")
                    return False
            # expected motion per burst ~ gain*speed*n*0.05; shrink the
            # burst so we do not overshoot the target
            per = gain * speed * 0.05
            k = int(max(1, min(n, round(dist / per))))
            v = err / dist * speed
            self.burst(v, n=k)
            print(f"  burst {i}: k={k} at {t.round(4)} err {dist*1000:.1f} mm")
        t, _ = self.tcp()
        print(f"servo_to: budget exhausted at {t.round(4)}")
        return False
