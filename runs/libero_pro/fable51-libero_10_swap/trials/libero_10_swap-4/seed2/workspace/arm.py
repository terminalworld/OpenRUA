#!/usr/bin/env python3
"""Persistent-client helper for the Panda: FK/IK, trajectories, gripper.

All poses are WORLD frame; converted to panda_link0 for MoveIt.
"""
import math
import sys
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
ARM = FJT["joints"]
# MoveIt's model frame IS world here (verified: FK of panda_link0 with
# empty frame_id returns (-0.51, 0, 0.42)); no offset needed.
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def down_quat(finger_axis_world):
    """Hand z pointing straight down, hand y (finger axis) along the
    given world direction (projected to horizontal)."""
    y = np.array(finger_axis_world, float)
    y[2] = 0
    y /= np.linalg.norm(y)
    z = np.array([0, 0, -1.0])
    x = np.cross(y, z)
    R = np.column_stack([x, y, z])
    return R_to_quat(R)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_ctl")
        self.js = {}
        self.wrench = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 10)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self.wrench.update(m=m), 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        for c, n in ((self.fjt, "fjt"), (self.grip, "grip")):
            if not c.wait_for_server(timeout_sec=20):
                raise SystemExit(f"no {n} server")
        for c, n in ((self.ik, "ik"), (self.fk, "fk")):
            if not c.wait_for_service(timeout_sec=20):
                raise SystemExit(f"no {n} service")
        self.spin_until(lambda: "m" in self.js, 10)

    def _on_js(self, m):
        self.js["m"] = m

    def spin_until(self, pred, timeout):
        end = time.time() + timeout
        while not pred() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return pred()

    def spin(self, n=5):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------- state ----------
    def joints(self):
        self.js.pop("m", None)
        self.spin_until(lambda: "m" in self.js, 10)
        m = self.js["m"]
        d = dict(zip(m.name, m.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def fk_pose(self, q=None, link="panda_hand"):
        """World-frame (pos, quat) of link for arm config q."""
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = np.array([p.orientation.x, p.orientation.y,
                         p.orientation.z, p.orientation.w])
        return pos, quat

    def tcp_pose(self, q=None):
        pos, quat = self.fk_pose(q)
        R = quat_to_R(quat)
        return pos + TCP * R[:, 2], quat

    # ---------- IK ----------
    def ik_solve(self, tcp_world, quat, seed=None, tries=3):
        """Joint config placing the TCP at tcp_world (world frame)."""
        R = quat_to_R(quat)
        hand_world = np.array(tcp_world, float) - TCP * R[:, 2]
        hand_base = hand_world - BASE_IN_WORLD
        seed = self.arm_q() if seed is None else seed
        for t in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            # group tip is panda_link8 (45 deg off panda_hand); be explicit
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, hand_base)
            (p.orientation.x, p.orientation.y,
             p.orientation.z, p.orientation.w) = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = \
                [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                q = [sol[j] for j in ARM]
                # sanity: FK of solution must match target
                pos, qq = self.tcp_pose(q)
                err = np.linalg.norm(pos - tcp_world)
                Rerr = np.abs(quat_to_R(qq) - R).max()
                if err < 0.005 and Rerr < 0.02:
                    return q
                print(f"  ik mismatch pos={err:.4f} rot={Rerr:.3f}", file=sys.stderr)
                seed = list(np.array(seed) + np.random.uniform(-0.2, 0.2, 7))
                continue
                print(f"  ik fk-mismatch {err:.4f}, retry", file=sys.stderr)
            else:
                print(f"  ik fail code={res and res.error_code.val}, retry",
                      file=sys.stderr)
            seed = list(np.array(seed) + np.random.uniform(-0.2, 0.2, 7))
        return None

    # ---------- motion ----------
    def move_q(self, q, seconds, verify=True, tol=0.005, resend=6):
        """Send trajectory; resend (controller lag) until joints converge."""
        for i in range(resend):
            code, err = self._move_once(q, seconds if i == 0 else 2.0, verify)
            if not verify or err < tol:
                return code, err
        return code, err

    def _move_once(self, q, seconds, verify=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        if verify:
            self.spin(5)
            cur = np.array(self.arm_q())
            err = np.abs(cur - np.array(q)).max()
            print(f"  move done code={code} max_joint_err={err:.4f}")
            return code, err
        return code, None

    def move_tcp(self, tcp_world, quat, seconds=3.0, seed=None):
        q = self.ik_solve(tcp_world, quat, seed)
        if q is None:
            print(f"  IK FAILED for {np.round(tcp_world,3)}")
            return None
        code, err = self.move_q(q, seconds)
        pos, _ = self.tcp_pose()
        print(f"  tcp now {np.round(pos,4)} (target {np.round(tcp_world,4)})")
        return q

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result
        self.spin(5)
        f = self.finger()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def force(self):
        self.wrench.pop("m", None)
        self.spin_until(lambda: "m" in self.wrench, 5)
        if "m" not in self.wrench:
            return None
        f = self.wrench["m"].wrench.force
        return np.array([f.x, f.y, f.z])
