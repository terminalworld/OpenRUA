#!/usr/bin/env python3
"""Small helper layer over MoveIt IK/FK + FollowJointTrajectory + gripper.

All poses are the panda_hand frame in the world frame (world = panda_link0
shifted by the base offset read from TF once).
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
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_axis(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.array([*(axis * math.sin(ang / 2)), math.cos(ang / 2)])


# hand pointing straight down, finger axis along world X
Q_DOWN_FX = np.array([math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0])
# hand pointing straight down, finger axis along world Y (home-like)
Q_DOWN_FY = np.array([1.0, 0.0, 0.0, 0.0])


def tilted(q_base, axis, ang):
    """Rotate orientation q_base by ang about a WORLD axis."""
    return quat_mul(quat_axis(axis, ang), q_base)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = {}
        self.wrench = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self.wrench.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        # machine fact (verified): /compute_fk and /compute_ik already work in
        # WORLD coordinates (FK of the current pose == TF world->panda_hand)
        self.base = np.zeros(3)
        self.spin(0.3)

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        if fresh:
            self.js.pop("m", None)
            while "m" not in self.js:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return abs(j["panda_finger_joint1"]), abs(j["panda_finger_joint2"])

    def force(self):
        self.wrench.pop("m", None)
        t0 = time.time()
        while "m" not in self.wrench and time.time() - t0 < 3:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if "m" not in self.wrench:
            return None
        f = self.wrench["m"].wrench.force
        return np.array([f.x, f.y, f.z])

    def hand_pose(self, q=None):
        """FK: world-frame position + quaternion of panda_hand."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = list(q if q is not None else self.arm_q())
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + self.base
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    # ---------------- planning ----------------
    def solve_ik(self, pos, quat, seed=None, attempts=3):
        """pos: world-frame panda_hand position. Returns joint list or None."""
        pos = np.asarray(pos, float) - self.base
        seed = list(seed if seed is not None else self.arm_q())
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            # the group's default tip is panda_link8 (45 deg off panda_hand)
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            req.ik_request.robot_state.joint_state.position = seed
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[n] for n in JOINTS]
        return None

    # ---------------- acting ----------------
    def move_joints(self, waypoints, seconds, verify=True):
        """waypoints: list of joint vectors (or one). seconds: total time."""
        if not isinstance(waypoints[0], (list, tuple, np.ndarray)):
            waypoints = [waypoints]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        n = len(waypoints)
        for i, wp in enumerate(waypoints):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = None
        if verify:
            q = np.array(self.arm_q())
            err = float(np.max(np.abs(q - np.array(waypoints[-1], float))))
        return code, err

    def move_to(self, pos, quat, seconds=3.0, seed=None):
        q = self.solve_ik(pos, quat, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for {np.round(pos,3)}")
        code, err = self.move_joints(q, seconds)
        return q, code, err

    def move_tcp(self, tcp_pos, quat, seconds=3.0, seed=None):
        """Place the fingertip point (TCP) at tcp_pos."""
        R = quat_R(quat)
        hand = np.asarray(tcp_pos, float) - TCP * R[:, 2]
        return self.move_to(hand, quat, seconds, seed)

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()


def line_ik(arm, tcp_from, tcp_to, quat, steps=4, seed=None, max_jump=0.6):
    """IK along a straight TCP line; returns list of joint waypoints (seeded
    consecutively) or raises if a solution is missing / jumps too far."""
    R = quat_R(quat)
    seed = list(seed if seed is not None else arm.arm_q())
    wps = []
    for i in range(1, steps + 1):
        tcp = np.asarray(tcp_from, float) + (np.asarray(tcp_to, float) - np.asarray(tcp_from, float)) * i / steps
        hand = tcp - TCP * R[:, 2]
        best = None
        for _ in range(6):
            q = arm.solve_ik(hand, quat, seed=seed, attempts=1)
            if q is None:
                continue
            jump = float(np.max(np.abs(np.array(q) - np.array(seed))))
            if best is None or jump < best[0]:
                best = (jump, q)
            if jump < max_jump:
                break
        if best is None:
            raise RuntimeError(f"IK failed at {np.round(tcp,3)}")
        if best[0] >= max_jump:
            raise RuntimeError(f"IK jump {best[0]:.2f} rad at {np.round(tcp,3)}")
        wps.append(best[1]); seed = best[1]
    return wps
