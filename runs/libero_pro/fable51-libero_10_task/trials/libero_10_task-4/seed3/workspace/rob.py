#!/usr/bin/env python3
"""Reusable robot helper for this Panda: joint state, TF, FK/IK, trajectory,
gripper, servo. Build once per process and reuse."""
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
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_IN_WORLD = np.zeros(3)  # verified: FK/IK with empty frame_id are in WORLD (model root)
from scipy.spatial.transform import Rotation as Rot
# IK tip link is panda_link8 = hand rotated -45deg about z (verified against FK)
R_HAND2LINK8 = Rot.from_euler("z", math.pi / 4)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_down_quat(yaw):
    """Hand z pointing down (world -z), hand x rotated by yaw about world z.
    q = Rz(yaw) * Rx(pi)."""
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sin(yaw/2),cos(yaw/2))
    s, c = math.sin(yaw / 2), math.cos(yaw / 2)
    # (c,0,0,s) * (1,0,0,0) quaternion product (w,x,y,z) convention:
    # q1=(w=c,x=0,y=0,z=s), q2=(w=0,x=1,y=0,z=0)
    w = c * 0 - 0 * 1 - 0 * 0 - s * 0
    x = c * 1 + 0 * 0 + 0 * 0 - s * 0
    y = c * 0 - 0 * 0 + 0 * 1 + s * 1
    z = c * 0 + 0 * 0 - 0 * 1 + s * 0
    return (x, y, z, w)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.wrench = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, next(
            s for s in M["sensors"] if s["kind"] == "wrench")["port"], self._on_w, 1)
        self.tf = Buffer()
        TransformListener(self.tf, self.node)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, m):
        self.js = m

    def _on_w(self, m):
        self.wrench = m

    def spin(self, n=3, dt=0.05):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=dt)

    # ---- sensing ----
    def joints(self):
        self.spin(3)
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in ARM]

    def fingers(self):
        self.spin(3)
        d = dict(zip(self.js.name, self.js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def hand_pose_world(self):
        """Hand pose in world from FK (fresh, not cached TF)."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        seed = JointState()
        seed.name = list(ARM)
        seed.position = self.joints()
        req.robot_state.joint_state = seed
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed: {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp_world(self):
        pos, q = self.hand_pose_world()
        R = quat_R(*q)
        return pos + M["hand"]["tcp_offset_m"] * R[:, 2], q

    def force(self):
        self.spin(3)
        f = self.wrench.wrench.force
        return np.array([f.x, f.y, f.z])

    # ---- IK ----
    def ik_solve(self, pos_world, quat, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pb = np.asarray(pos_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, pb)
        q8 = (Rot.from_quat(list(quat)) * R_HAND2LINK8).as_quat()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)
        req.ik_request.avoid_collisions = False
        s = JointState()
        s.name = list(ARM)
        s.position = list(seed if seed is not None else self.joints())
        req.ik_request.robot_state.joint_state = s
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None:
            raise RuntimeError("IK no answer")
        if r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp(self, tcp_world, quat, seed=None):
        R = quat_R(*quat)
        hand = np.asarray(tcp_world) - M["hand"]["tcp_offset_m"] * R[:, 2]
        return self.ik_solve(hand, quat, seed)

    # ---- action ----
    def move_joints(self, positions, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = list(waypoints or []) + [positions]
        for i, w in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(x) for x in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.joints()) - np.array(positions)).max()
        return code, err

    def move_tcp(self, tcp_world, quat, seconds=3.0, seed=None):
        sol = self.ik_tcp(tcp_world, quat, seed)
        if sol is None:
            raise RuntimeError(f"IK failed for tcp {tcp_world}")
        code, err = self.move_joints(sol, seconds)
        pos, _ = self.tcp_world()
        return code, err, pos

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def servo(self, vx=0.0, vy=0.0, vz=0.0, ticks=20):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
