#!/usr/bin/env python3
"""Small persistent control library for the Panda on this machine."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
TCP = M["hand"]["tcp_offset_m"]
BASE = np.array([0.0, 0.0, 0.0])  # MoveIt FK/IK poses are already in world coords (verified vs DH FK)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def hand_quat(psi):
    """link8 orientation quaternion for a down-pointing hand whose finger
    axis is rotated psi about world z from world y (psi=0: fingers along y,
    psi=pi/2: fingers along x). panda_hand is link8 yawed by +45 deg."""
    return down_quat(psi - math.pi / 4)


def down_quat(yaw):
    """Hand z pointing down (world -z), fingers opening along a direction
    rotated by `yaw` about world z from world x... quaternion (x,y,z,w).
    Rotation = Rz(yaw) * Rx(pi)."""
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sin(yaw/2),cos(yaw/2))
    s, c = math.sin(yaw / 2), math.cos(yaw / 2)
    # q = qz * qx with qz=(w=c, z=s), qx=(w=0, x=1)  ->  (x=c, y=s, z=0, w=0)
    return (c, s, 0.0, 0.0)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_" + str(int(time.time()) % 100000))
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)
        self.wait_js()

    def _on_js(self, m):
        self.js = dict(zip(m.name, m.position))
        self.js_t = time.time()

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        self.js = {}
        while not self.js:
            self.spin(0.2)
        return self.js

    def arm_q(self):
        self.wait_js()
        return [self.js[j] for j in ARM]

    def fingers(self):
        self.wait_js()
        return self.js["panda_finger_joint1"], self.js["panda_finger_joint2"]

    # ---- FK / IK (base frame = panda_link0; world = base + BASE) ----
    def fk(self, q=None, link="panda_link8"):
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q_ = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, q_

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = quat_R(*quat)
        return pos + TCP * R[:, 2], quat

    def ik(self, pos_world, quat, seed=None, at_tcp=True, timeout=1.0):
        pos = np.array(pos_world, dtype=float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        pos = pos - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        seed = self.arm_q() if seed is None else seed
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    # ---- motion ----
    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                t = seconds * (i + 1) / n
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        self.wait_js()
        err = max(abs(a - b) for a, b in zip(self.arm_q(), q))
        print(f"move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik(pos_world, quat, seed=seed)
        if q is None:
            print(f"IK FAILED for {np.round(pos_world,3)}")
            return None
        return self.move_q(q, seconds)

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        self.wait_js()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}")
        return r

    def servo(self, lin=(0, 0, 0), ang=(0, 0, 0), n=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            self.spin(dt)
        self.wait_js()


def go(r, pos, psi, seconds=2.5, tries=3, tol=0.02):
    """IK + trajectory to TCP pos with finger axis psi; resend on lag."""
    q = r.ik(pos, hand_quat(psi))
    if q is None:
        print(f"IK FAILED for {np.round(pos, 3)} psi={psi}")
        return False
    for i in range(tries):
        code, err = r.move_q(q, seconds)
        if err < tol:
            break
    tcp, _ = r.tcp()
    print(f"go -> tcp {np.round(tcp, 4)} (target {np.round(pos, 4)}) err={np.linalg.norm(tcp - np.array(pos)):.4f}")
    return err < tol
