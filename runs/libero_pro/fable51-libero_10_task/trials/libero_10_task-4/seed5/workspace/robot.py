#!/usr/bin/env python3
"""Small helper library for the Panda: joints, FK, IK, trajectories, gripper, servo.

All poses are in the WORLD frame: MoveIt's model frame here is `world`
(verified: FK of panda_link0 returns (-0.51, 0, 0.42)), so IK/FK poses with
an empty frame_id are world-frame poses.
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
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.510, 0.0, 0.420])
TCP = float(M["hand"]["tcp_offset_m"])


def w2b(p):
    return np.asarray(p, float) - BASE_IN_WORLD


def b2w(p):
    return np.asarray(p, float) + BASE_IN_WORLD


def quat_R(x, y, z, w):
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


def down_quat(yaw_deg=0.0):
    """Hand z pointing down (-Z base), fingers rotated by yaw about vertical.
    yaw=0 -> hand x axis along base +x (fingers close along base y)."""
    # rotation 180 deg about x: hand z -> -z
    q_flip = np.array([1.0, 0.0, 0.0, 0.0])
    yaw = math.radians(yaw_deg)
    q_yaw = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    return quat_mul(q_yaw, q_flip)


class Robot:
    def __init__(self, name="robot_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] + abs(d["panda_finger_joint2"])

    def wrench(self):
        self._wr = None
        while self._wr is None:
            self.spin(0.1)
        f = self._wr.wrench.force
        t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def fk(self, q=None, link="panda_hand"):
        """Return (pos, quat[x,y,z,w]) of link in world frame."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp(self, q=None):
        """TCP (fingertip centre) position in world frame."""
        p, qt = self.fk(q)
        R = quat_R(*qt)
        return p + TCP * R[:, 2], qt

    # ---------- planning ----------
    def ik(self, pos, quat, seed=None, at_tcp=True, attempts=1):
        pos = np.asarray(pos, float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        # the IK tip link is panda_link8, which is panda_hand rotated +45deg
        # about z (hand = link8 * Rz(-45)); convert the hand quat to link8
        quat = quat_mul(quat, np.array([0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8)]))
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        for _ in range(attempts):
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        raise RuntimeError(f"IK failed ({res and res.error_code.val}) for {pos} {quat}")

    # ---------- action ----------
    def move_q(self, q, seconds=3.0, tol=0.01, retries=2):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        for i in range(retries + 1):
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            gh = send.result()
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            print(f"  move_q: code={code} max_err={err:.4f}", flush=True)
            if err < tol:
                return True
        return err < tol

    def move_qs(self, qs, seconds_each=2.0):
        """Multi-point trajectory."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        t = 0.0
        for q in qs:
            t += seconds_each
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(qs[-1])).max()
        print(f"  move_qs: code={code} max_err={err:.4f}", flush=True)
        return err < 0.01

    def move_tcp(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed, at_tcp=True)
        ok = self.move_q(q, seconds)
        p, _ = self.tcp()
        print(f"  tcp now {p.round(4)} target {np.asarray(pos).round(4)}", flush=True)
        return ok

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap

    def open(self):
        return self.gripper(GRIP["open_m"])

    def close(self):
        return self.gripper(GRIP["closed_m"])

    def servo(self, v, n=20, dt=0.05):
        """Stream a base-frame linear velocity v (m/s) for n ticks."""
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            self.spin(dt)
        p, _ = self.tcp()
        print(f"  servo done, tcp {p.round(4)}", flush=True)
        return p
