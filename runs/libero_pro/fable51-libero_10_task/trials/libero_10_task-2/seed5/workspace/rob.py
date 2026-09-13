#!/usr/bin/env python3
"""Reusable robot helpers for this Panda workstation (one node, reused clients).

World frame = panda_link0 shifted by BASE (world->panda_link0 from TF).
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
BASE = np.array([-0.660, 0.000, 0.912])  # world -> panda_link0 translation
TCP = float(M["hand"]["tcp_offset_m"])


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def down_quat(yaw):
    """Hand pointing straight down (hand z = world -z), hand x rotated by yaw
    about world z. yaw=0 -> hand x = world +x, fingers close along world y."""
    # q = Rz(yaw) * Rx(pi)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return quat_mul((0.0, 0.0, sy, cy), (1.0, 0.0, 0.0, 0.0))


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.wrench = None
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._w_cb, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk.wait_for_service(10)

    def _js_cb(self, m):
        self.js = m

    def _w_cb(self, m):
        self.wrench = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self.js = None
            while self.js is None:
                self.spin(0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def hand_pose(self, q=None):
        """FK: returns (pos_world[3], quat[4] xyzw) of panda_hand."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])  # FK already reports world
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, quat

    def tcp_pose(self, q=None):
        pos, quat = self.hand_pose(q)
        R = quat_to_R(quat)
        return pos + TCP * R[:, 2], quat

    # ---------- planning ----------
    def ik(self, pos_world, quat, seed=None, at_tcp=False, timeout=60):
        pos = np.array(pos_world, dtype=float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(quat)[:, 2]
        # IK tip link is panda_link8 (= hand rotated +45deg about z); poses
        # are interpreted in the same world frame FK reports (verified).
        quat = quat_mul(quat, (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8)))
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in (seed or self.arm_q())]
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, via=None, retries=3, tol=0.01):
        for attempt in range(retries):
            code, err = self._move_q_once(q, seconds, via if attempt == 0 else None)
            if err <= tol:
                break
            print(f"move_q: retry {attempt + 1} (err {err:.4f})", flush=True)
        return code, err

    def _move_q_once(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [q]
        n = len(wps)
        for i, wp in enumerate(wps):
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            t = seconds * (i + 1) / n
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        r = res.result()
        code = r.result.error_code if r else None
        err = float(np.max(np.abs(np.array(self.arm_q()) - np.array(q))))
        print(f"move_q: error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_pose(self, pos_world, quat, seconds=3.0, at_tcp=False, seed=None):
        q = self.ik(pos_world, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            print("move_pose: IK FAILED, no motion", flush=True)
            return None
        code, err = self.move_q(q, seconds)
        p, _ = (self.tcp_pose() if at_tcp else self.hand_pose())
        print(f"move_pose: target={np.round(pos_world,4)} actual={np.round(p,4)}", flush=True)
        return q

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result
        gap = self.finger_gap()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap

    def servo(self, v=(0, 0, 0), w=(0, 0, 0), n=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, w)
        for _ in range(n):
            self.tw_pub.publish(msg)
            self.spin(dt)
        return self.hand_pose()


if __name__ == "__main__":
    r = Robot()
    q = r.arm_q()
    print("q:", np.round(q, 4))
    print("finger gap:", r.finger_gap())
    pos, quat = r.hand_pose(q)
    print("hand world pos:", np.round(pos, 4), "quat:", np.round(quat, 4))
    tp, _ = r.tcp_pose(q)
    print("tcp world pos:", np.round(tp, 4))
    # IK round trip check
    sol = r.ik(pos, quat)
    print("ik roundtrip:", None if sol is None else np.round(sol, 4))
