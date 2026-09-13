#!/usr/bin/env python3
"""Reusable controller: joint state, FK, IK, trajectory, gripper.

All poses: position of the TCP (fingertip midpoint) unless hand=True.
Orientation: quaternion (x,y,z,w) of the hand frame. TOP_DOWN points the
fingers at the table with the finger axis along world y.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

TOP_DOWN = (1.0, 0.0, 0.0, 0.0)


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def qz(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def top_down_yaw(deg):
    """Top-down grasp with finger axis rotated `deg` about world z."""
    return qmul(qz(deg), TOP_DOWN)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self):
        self.m = yaml.safe_load(open("/workspace/machine.yaml"))
        self.traj = next(a for a in self.m["actuators"] if a["kind"] == "joint_trajectory")
        self.grip = next(a for a in self.m["actuators"] if a["kind"] == "gripper")
        self.joints = self.traj["joints"]
        self.tcp_off = float(self.m["hand"]["tcp_offset_m"])
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_ctl")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, self.traj["port"])
        self.gc = ActionClient(self.node, GripperCommand, self.grip["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, self.m["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(timeout_sec=20), "no fjt server"
        assert self.gc.wait_for_server(timeout_sec=20), "no gripper server"
        assert self.ik_cli.wait_for_service(timeout_sec=20), "no ik"
        assert self.fk_cli.wait_for_service(timeout_sec=20), "no fk"
        self.wait_js()

    def _on_js(self, msg):
        self._js = msg

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        js = self.wait_js()
        return [js[j] for j in self.joints]

    def fingers(self):
        js = self.wait_js()
        return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    # ---- kinematics -------------------------------------------------
    def fk(self, q=None):
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(self.joints)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, quat

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = quat_to_R(quat)
        return pos + self.tcp_off * R[:, 2], quat

    def ik(self, pos, quat, seed=None, hand=False, tries=3):
        pos = np.array(pos, dtype=float)
        if not hand:
            R = quat_to_R(quat)
            pos = pos - self.tcp_off * R[:, 2]
        seed = seed if seed is not None else self.arm_q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = self.m["planning"]["group"]
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(self.joints)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.timeout.sec = 2
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in self.joints]
            print(f"  IK attempt failed: {None if res is None else res.error_code.val}", flush=True)
        return None

    # ---- motion -----------------------------------------------------
    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joints)
        pts = []
        wps = list(waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        assert gh is not None and gh.accepted, "fjt goal rejected"
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_tcp(self, pos, quat, seconds=3.0, seed=None, retries=3):
        q = self.ik(pos, quat, seed=seed)
        if q is None:
            print(f"  NO IK for {np.round(pos,3)}", flush=True)
            return None
        for i in range(retries + 1):
            code, err = self.move_q(q, seconds)
            if code == 0 and err < 0.02:
                break
            print(f"  retry {i+1}: tolerance violation, resending", flush=True)
        tcp, _ = self.tcp()
        print(f"  tcp now {np.round(tcp,4)} target {np.round(pos,4)} err {np.linalg.norm(tcp-pos):.4f}", flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(self.grip["max_effort"])
        fut = self.gc.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def open(self):
        return self.gripper(self.grip["open_m"])

    def close(self):
        return self.gripper(self.grip["closed_m"])
