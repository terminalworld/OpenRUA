#!/usr/bin/env python3
"""Reusable helpers for this Panda: joint state, FK, IK, trajectories, gripper.

All poses are in the arm base frame (panda_link0) unless noted; world =
base + (-0.51, 0, 0.42) (from /tf). Import and use, or run as a module
for quick one-offs.
"""
import math
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parent
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
LIMITS = FJT["limits_rad"]
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # FK(panda_link0) = (-0.51,0,0.42): model frame IS world
TCP_OFF = float(M["hand"]["tcp_offset_m"])


def world_to_base(p):
    return np.asarray(p, float) - BASE_IN_WORLD


def base_to_world(p):
    return np.asarray(p, float) + BASE_IN_WORLD


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def down_quat(yaw=0.0):
    """Hand pointing straight down (hand +Z = world -Z); `yaw` rotates the
    finger-opening axis about vertical. yaw=0 -> fingers open along base Y."""
    q_down = np.array([1.0, 0.0, 0.0, 0.0])  # 180 deg about X
    q_yaw = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    return quat_mul(q_yaw, q_down)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)

    # ---------------- sensing ----------------
    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"])

    def fk_pose(self, q=None, link="panda_hand"):
        """Returns (pos[3], quat[4]) of `link` in base frame."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp_world(self, q=None):
        pos, quat = self.fk_pose(q)
        R = quat_to_R(*quat)
        return base_to_world(pos + TCP_OFF * R[:, 2])

    # ---------------- IK ----------------
    def solve_ik(self, pos_base, quat, seed=None, at_tcp=False, timeout=20.0):
        """IK for the hand (or TCP if at_tcp) pose in base frame. Returns q list or None."""
        pos = np.asarray(pos_base, float)
        if at_tcp:
            R = quat_to_R(*quat)
            pos = pos - TCP_OFF * R[:, 2]
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
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout + 30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in ARM]

    def ik_world(self, pos_world, quat, seed=None, at_tcp=True, tries=3):
        q = None
        s = seed
        for _ in range(tries):
            q = self.solve_ik(world_to_base(pos_world), quat, seed=s, at_tcp=at_tcp)
            if q is not None:
                break
        return q

    # ---------------- motion ----------------
    def move_joints(self, points, times):
        """points: list of q lists; times: cumulative seconds per point."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(points, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        r = res.result()
        code = r.result.error_code if r else None
        q_now = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q_now, points[-1]))
        return code, err

    def move_q(self, q, seconds=3.0, settle=True):
        code, err = self.move_joints([q], [seconds])
        if settle and (code != 0 or err > 0.02):
            for _ in range(3):
                code, err = self.move_joints([q], [2.0])
                if code == 0 and err <= 0.02:
                    break
        return code, err

    def move_world(self, pos_world, quat, seconds=3.0, at_tcp=True, seed=None):
        q = self.ik_world(pos_world, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            raise RuntimeError(f"IK failed for world {pos_world}")
        code, err = self.move_q(q, seconds)
        return q, code, err

    def move_line_world(self, p_from, p_to, quat, n=6, seconds=3.0, at_tcp=True,
                        max_jump=0.4, settle=True):
        """Straight TCP line via IK waypoints in one trajectory. Rejects
        waypoints whose joint solution jumps branches; re-sends the final
        point if the controller lagged (tolerance violation)."""
        pts, times = [], []
        seed = self.arm_q()
        prev = seed
        for i in range(1, n + 1):
            p = np.asarray(p_from) + (np.asarray(p_to) - np.asarray(p_from)) * i / n
            q = None
            for _ in range(5):
                cand = self.ik_world(p, quat, seed=prev, at_tcp=at_tcp, tries=1)
                if cand is not None and max(abs(a - b) for a, b in zip(cand, prev)) <= max_jump:
                    q = cand
                    break
            if q is None:
                raise RuntimeError(f"IK failed / branch jump on line at {p}")
            prev = q
            pts.append(q)
            times.append(seconds * i / n)
        code, err = self.move_joints(pts, times)
        if settle and (code != 0 or err > 0.02):
            for _ in range(3):
                code, err = self.move_joints([pts[-1]], [2.0])
                if code == 0 and err <= 0.02:
                    break
        return code, err

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result
        return r.reached_goal, r.stalled, self.finger_gap()

    def open(self):
        return self.gripper(GRIP["open_m"])

    def close(self):
        return self.gripper(GRIP["closed_m"])
