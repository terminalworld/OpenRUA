#!/usr/bin/env python3
"""Reusable robot helpers: joint state, FK, IK, trajectory, gripper, twist."""
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
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # world->panda_link0 (from TF)
TCP = M["hand"]["tcp_offset_m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


# machine fact (measured): /compute_ik solves for panda_link8, which is the
# hand frame rotated +45 deg about z; positions are in WORLD coordinates.
HAND_TO_LINK8 = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))


def yaw_down_quat(yaw):
    """Hand pointing straight down (hand z = -world z), hand x rotated by yaw about world z.
    yaw=0: hand x = world x, hand y = -world y (fingers close along world y)."""
    # q = Rz(yaw) * Rx(pi)
    cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sz,cz); product Rz*Rx:
    # (w1w2 - v1.v2, w1 v2 + w2 v1 + v1 x v2)
    w = cz * 0 - (0 * 1 + 0 * 0 + sz * 0)
    v = cz * np.array([1, 0, 0]) + 0 * np.array([0, 0, sz]) + np.cross([0, 0, sz], [1, 0, 0])
    return (v[0], v[1], v[2], w)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(timeout_sec=20)
        self.grip.wait_for_server(timeout_sec=20)
        self.ik_cli.wait_for_service(timeout_sec=20)
        self.fk_cli.wait_for_service(timeout_sec=20)
        while self._js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, m):
        self._js = m

    def spin(self, n=3):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def joints(self):
        self._js = None
        while self._js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger_gap(self):
        d = self.joints()
        return abs(d["panda_finger_joint1"]) + abs(d["panda_finger_joint2"])

    def _seed(self, q=None):
        js = JointState()
        q = q if q is not None else self.arm_q()
        js.name = list(ARM)
        js.position = [float(v) for v in q]
        return js

    def fk(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        ps = res.pose_stamped[0]
        p, o = ps.pose.position, ps.pose.orientation
        return np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w]), ps.header.frame_id

    def ik(self, pos, quat, seed=None, at_tcp=False, attempts=3):
        """pos in the frame IK uses (see test); quat (x,y,z,w). Returns arm q or None."""
        pos = np.array(pos, dtype=float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        quat = qmul(quat, HAND_TO_LINK8)  # hand orientation -> link8 orientation
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state = self._seed(seed)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=2)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        print("IK failed", None if res is None else res.error_code.val)
        return None

    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (waypoints or []) + [q]
        for i, w in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(v) for v in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.max(np.abs(np.array(self.arm_q()) - np.array(q)))
        print(f"move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_pose(self, pos, quat, seconds=3.0, at_tcp=True, seed=None):
        q = self.ik(pos, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            return None
        return self.move_q(q, seconds)

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap

    def twist(self, lin, ang=(0, 0, 0), ticks=20):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def tcp_world(self):
        p, q, f = self.fk()
        R = quat_R(*q)
        return p + TCP * R[:, 2], q, f
