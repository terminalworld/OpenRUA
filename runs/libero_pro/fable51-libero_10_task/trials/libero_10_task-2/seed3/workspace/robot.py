#!/usr/bin/env python3
"""Reusable controller: FK/IK/trajectory/gripper with persistent clients.

World frame = panda_link0 + BASE offset (from TF, world->panda_link0).
All public poses are WORLD-frame TCP poses (fingertip centre).
"""
import math
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

BASE = np.array([0.0, 0.0, 0.0])  # FK/IK already answer in the world frame (verified vs cameras)


def quat_down(yaw_deg=0.0):
    """Hand pointing straight down (hand +Z = world -Z), fingers closing
    along world Y for yaw=0; yaw rotates about world Z (deg)."""
    r = Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    return r.as_quat()  # x,y,z,w


class Robot:
    def __init__(self):
        root = Path(__file__).resolve().parent
        self.M = yaml.safe_load((root / "machine.yaml").read_text())
        self.traj = next(a for a in self.M["actuators"] if a["kind"] == "joint_trajectory")
        self.grip = next(a for a in self.M["actuators"] if a["kind"] == "gripper")
        self.joints = self.traj["joints"]
        self.tcp_off = float(self.M["hand"]["tcp_offset_m"])
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_ctl")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, self.traj["port"])
        self.gc = ActionClient(self.node, GripperCommand, self.grip["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, self.M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.gc.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def _on_js(self, m):
        self._js = m

    # ---------- sensing ----------
    def joint_state(self, fresh=True):
        if fresh:
            self._js = None
        t0 = time.time()
        while self._js is None and time.time() - t0 < 10:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        js = self.joint_state()
        return [js[j] for j in self.joints]

    def finger(self):
        js = self.joint_state()
        return js.get("panda_finger_joint1")

    def fk(self, q=None):
        """World-frame TCP pose (pos, quat xyzw) for joint vector q."""
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(self.joints)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        R = Rot.from_quat(quat).as_matrix()
        tcp = pos + self.tcp_off * R[:, 2]
        return tcp, quat

    # ---------- planning ----------
    def ik(self, tcp_world, quat, seed=None):
        quat = np.asarray(quat, float)
        R = Rot.from_quat(quat).as_matrix()
        hand = np.asarray(tcp_world, float) - self.tcp_off * R[:, 2] - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = self.arm_q() if seed is None else seed
        req.ik_request.robot_state.joint_state.name = list(self.joints)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"IK failed: {None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in self.joints]

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joints)
        pts = []
        if waypoints:
            for (wq, wt) in waypoints:
                pt = JointTrajectoryPoint(positions=[float(v) for v in wq])
                pt.time_from_start = Duration(sec=int(wt), nanosec=int((wt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        qn = self.arm_q()
        err = max(abs(a - b) for a, b in zip(qn, q))
        print(f"  move_q done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, tcp_world, quat, seconds=3.0, seed=None):
        q = self.ik(tcp_world, quat, seed)
        code, err = self.move_q(q, seconds)
        tcp, _ = self.fk()
        print(f"  tcp now {np.round(tcp, 4)} target {np.round(tcp_world, 4)}")
        return q

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(self.grip.get("max_effort", 30.0))
        fut = self.gc.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result
        f = self.finger()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} finger={f:.4f}")
        return f

    def open(self):
        return self.gripper(self.grip["open_m"])

    def close(self):
        return self.gripper(self.grip["closed_m"])
