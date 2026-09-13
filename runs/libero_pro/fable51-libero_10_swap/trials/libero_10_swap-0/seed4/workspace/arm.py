#!/usr/bin/env python3
"""Reusable arm helpers: one node, persistent clients (IK, FK, FJT, gripper).

World <-> base: panda_link0 sits at world (-0.51, 0, 0.42), identity rotation.
All public functions take WORLD coordinates for the TCP (fingertip point).
"""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM_JOINTS = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
# Verified live: /compute_fk and /compute_ik (empty frame_id) both use the
# WORLD frame on this machine (FK matched tf world->panda_hand), so no
# base offset is applied. panda_link0 sits at world (-0.51, 0, 0.42).
BASE_IN_WORLD = np.zeros(3)
# top-down grasp: hand z down, hand x along world +x (fingers close along world y)
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def yaw_q(yaw):
    """Top-down orientation rotated by yaw about world z (fingers close along
    the axis at yaw+90deg)."""
    # q = Rz(yaw) * (1,0,0,0)
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    # Rz(yaw) = (0,0,s,c); product (0,0,s,c)*(1,0,0,0)
    return (c, s, 0.0, 0.0)


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_ctl")
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fk.wait_for_service(10), "no FK"
        assert self.fjt.wait_for_server(10), "no FJT"
        assert self.grip.wait_for_server(10), "no gripper"
        self.spin_until(lambda: self._js is not None, 10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin_until(self, cond, timeout):
        end = time.time() + timeout
        while not cond() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return cond()

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            self.spin_until(lambda: self._js is not None, 10)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM_JOINTS]

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] + abs(d["panda_finger_joint2"])

    def wrench(self):
        self._wr = None
        self.spin_until(lambda: self._wr is not None, 5)
        f = self._wr.wrench.force
        return (f.x, f.y, f.z)

    # ---- kinematics -------------------------------------------------
    def fk_hand_world(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM_JOINTS
        req.robot_state.joint_state.position = q or self.arm_q()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), r.pose_stamped[0].header.frame_id

    def ik_tcp_world(self, xyz, quat=Q_DOWN, seed=None):
        """IK for the TCP at world xyz with the given hand orientation.
        Returns joint list or None."""
        R = quat_R(*quat)
        hand = np.array(xyz) - TCP_OFF * R[:, 2]   # TCP is +Z of hand
        base = hand - BASE_IN_WORLD
        # Verified live: /compute_ik solves for a tip link yawed -45deg from
        # panda_hand (link8), so the resulting panda_hand orientation comes
        # out +45deg about its z from the request. Pre-rotate by -45deg.
        quat = quat_mul(quat, (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8)))
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        req.ik_request.robot_state.joint_state.name = ARM_JOINTS
        req.ik_request.robot_state.joint_state.position = seed or self.arm_q()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print("IK failed", None if r is None else r.error_code.val, "for", xyz)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM_JOINTS]

    # ---- motion -----------------------------------------------------
    def move_joints(self, waypoints, seconds):
        """waypoints: list of joint lists; seconds: list of cumulative times."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        for q, t in zip(waypoints, seconds):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = max(abs(a - b) for a, b in zip(self.arm_q(), waypoints[-1]))
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        # The sim arm can lag the trajectory clock: the result comes back
        # (code -5) with the arm frozen short of the goal. Re-send the final
        # point alone, slow enough for the remaining error, until it converges.
        for _ in range(4):
            if err < 0.01:
                break
            goal.trajectory.points = [JointTrajectoryPoint(
                positions=[float(v) for v in waypoints[-1]],
                time_from_start=Duration(sec=int(max(2.0, err / 0.4)) + 1))]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            err = max(abs(a - b) for a, b in zip(self.arm_q(), waypoints[-1]))
            print(f"  resend code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, xyz, quat=Q_DOWN, seconds=3.0, via=None):
        """IK then trajectory. via: optional list of intermediate TCP xyz."""
        seed = self.arm_q()
        wps, ts = [], []
        pts = (via or []) + [list(xyz)]
        for i, p in enumerate(pts):
            q = self.ik_tcp_world(p, quat, seed)
            if q is None:
                return None
            wps.append(q)
            seed = q
            ts.append(seconds * (i + 1) / len(pts))
        code, err = self.move_joints(wps, ts)
        pos, _, _ = self.fk_hand_world()
        R = quat_R(*quat)
        tcp = pos + TCP_OFF * R[:, 2]
        print(f"  tcp now {tcp.round(4)} target {np.array(xyz).round(4)}")
        return tcp

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        gap = self.finger_gap()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap


def quat_mul(a, b):
    """Hamilton product a*b, quaternions as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
