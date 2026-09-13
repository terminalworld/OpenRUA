#!/usr/bin/env python3
"""Arm helper library: joint state, FK, IK, trajectory, gripper.

import arm; a = arm.Arm(); a.move_pose(x,y,z, R=..., seconds=3)
World frame here == panda_link0 frame shifted by BASE (machine facts).
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
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
# Verified empirically: /compute_fk and /compute_ik with frame_id "" use the
# WORLD frame on this machine (FK of the current q == TF world->panda_hand,
# and IK of the world-frame hand pose reproduces the current q).
BASE = np.array([0.0, 0.0, 0.0])
TCP = M["hand"]["tcp_offset_m"]


def R_to_quat(R):
    """Rotation matrix -> (x, y, z, w)."""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()


def quat_to_R(q):
    from scipy.spatial.transform import Rotation
    return Rotation.from_quat(q).as_matrix()


def R_from_axes(z, x):
    """Rotation whose hand +Z (approach) is z and hand +X is x (both world)."""
    z = np.asarray(z, float); z /= np.linalg.norm(z)
    x = np.asarray(x, float); x = x - z * (x @ z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


# Hand pointing straight down; fingers open along world Y (as at start).
R_DOWN_FY = R_from_axes([0, 0, -1], [1, 0, 0])
# Hand pointing straight down; fingers open along world X.
R_DOWN_FX = R_from_axes([0, 0, -1], [0, 1, 0])


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self):
        """dict name->position (fresh)."""
        self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        while self._wr is None:
            self.spin(0.2)
        f = self._wr.wrench.force
        return np.array([f.x, f.y, f.z])

    # ---------- kinematics ----------
    def fk_hand(self, q=None):
        """Hand pose in WORLD: (pos, R)."""
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(map(float, q))
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        R = quat_to_R([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, R

    def tcp(self, q=None):
        pos, R = self.fk_hand(q)
        return pos + TCP * R[:, 2], R

    def solve_ik(self, pos_world, R, seed=None, attempts=3):
        """Hand pose (world) -> arm joint list, or None."""
        p = np.asarray(pos_world, float) - BASE
        q = R_to_quat(R)
        seed = self.arm_q() if seed is None else seed
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            # the group tip is panda_link8 (45 deg yawed from the hand);
            # ask for the hand link explicitly so R means the hand frame
            req.ik_request.ik_link_name = "panda_hand"
            ps = req.ik_request.pose_stamped.pose
            ps.position.x, ps.position.y, ps.position.z = map(float, p)
            ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, q)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            req.ik_request.robot_state.joint_state.position = list(map(float, seed))
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
            print(f"  IK attempt failed: {res and res.error_code.val}", file=sys.stderr)
        return None

    # ---------- motion ----------
    def move_joints(self, q, seconds=3.0, via=None, tol=0.02, retries=3):
        """Send trajectory and wait; resend on lag (machine fact: -5 is
        usually controller lag, resending converges)."""
        code, err = self._send_traj(q, seconds, via)
        for _ in range(retries):
            if err <= tol:
                break
            code, err = self._send_traj(q, max(2.0, seconds * 0.6))
        return code, err

    def _send_traj(self, q, seconds, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        wps = (via or []) + [q]
        for i, w in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=list(map(float, w)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.max(np.abs(np.array(self.arm_q()) - np.array(q)))
        print(f"  traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_pose(self, pos_world, R, seconds=3.0, seed=None):
        q = self.solve_ik(pos_world, R, seed=seed)
        if q is None:
            print("  IK FAILED, no motion"); return None
        code, err = self.move_joints(q, seconds)
        p, _ = self.fk_hand()
        print(f"  hand now at {np.round(p, 4)} (target {np.round(pos_world, 4)})")
        return q

    def move_tcp(self, tcp_world, R, seconds=3.0, seed=None):
        """Place the fingertip centre (TCP) at tcp_world with orientation R."""
        hand = np.asarray(tcp_world, float) - TCP * R[:, 2]
        return self.move_pose(hand, R, seconds, seed)

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap
