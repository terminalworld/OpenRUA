#!/usr/bin/env python3
"""Small helper library: one node, reusable clients for FK / IK /
trajectory / gripper / servo, joint-state and wrench readers.
World frame = panda_link0 + (-0.51, 0, 0.42) (from tf2_echo)."""
import math
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
WORLD_T_BASE = np.array([-0.51, 0.0, 0.42])  # tf world->panda_link0
# NOTE: /compute_fk and /compute_ik on this machine report/accept poses in
# the WORLD frame already (verified: FK of the ready pose = base pose + offset).
TCP = float(M["hand"]["tcp_offset_m"])

# top-down grasp orientation (hand z down, fingers along base y)
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def q_yaw_down(yaw):
    """Hand pointing down, fingers rotated by yaw about vertical."""
    # R = Rz(yaw) * Rx(pi)
    qz = (0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
    qx = (1, 0, 0, 0)
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        assert self.fjt.wait_for_server(10), "no fjt"
        assert self.grip.wait_for_server(10), "no gripper"
        assert self.ik.wait_for_service(10), "no ik"
        assert self.fk.wait_for_service(10), "no fk"
        while self._js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, n=3):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self.spin()
        d = dict(zip(self._js.name, self._js.position))
        return [d[j] for j in ARM]

    def fingers(self):
        self.spin()
        d = dict(zip(self._js.name, self._js.position))
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def wrench(self):
        self.spin()
        if self._wr is None:
            return None
        f = self._wr.wrench.force
        return np.array([f.x, f.y, f.z])

    def _seed(self, q=None):
        js = JointState()
        js.name = list(ARM)
        js.position = list(q if q is not None else self.joints())
        return js

    def fk_pose(self, q=None):
        """Hand pose in base frame: (xyz, quat xyzw)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                (p.orientation.x, p.orientation.y, p.orientation.z,
                 p.orientation.w))

    def tcp_world(self, q=None):
        p, quat = self.fk_pose(q)
        R = quat_R(*quat)
        return p + R[:, 2] * TCP

    def ik_hand_base(self, xyz, quat, seed=None, tries=3):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        (p.orientation.x, p.orientation.y, p.orientation.z,
         p.orientation.w) = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        for _ in range(tries):
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    def ik_tcp_world(self, xyz_world, quat=Q_DOWN, seed=None):
        """IK for the TCP (fingertip centre) at a WORLD position."""
        R = quat_R(*quat)
        hand = np.array(xyz_world) - R[:, 2] * TCP
        return self.ik_hand_base(hand, quat, seed)

    def move(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = seconds * (i + 1) / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.max(np.abs(np.array(self.joints()) - np.array(q)))
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def servo(self, v, n=20):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            pub_ok = self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
