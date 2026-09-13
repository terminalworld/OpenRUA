#!/usr/bin/env python3
"""Reusable motion/perception helpers for this Panda (clients built once)."""
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
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
TCP = M["hand"]["tcp_offset_m"]
# verified: /compute_fk output matches TF world->panda_hand exactly, so the
# planner's model frame IS world here (no base offset to apply)
BASE_IN_WORLD = np.zeros(3)

# top-down grasp orientation with the finger-closing axis yawed by psi
# (psi=0: fingers close along world y; psi=90deg: along world x)
def down_quat(psi_deg=0.0):
    h = math.radians(psi_deg) / 2
    return (math.cos(h), math.sin(h), 0.0, 0.0)  # x y z w


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("robot_lib")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, m):
        self.js = m

    def spin(self, t=0.1):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    # ---------- sensing ----------
    def joints(self):
        self.spin(0.2)
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in ARM]

    def fingers(self):
        self.spin(0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def hand_pose(self):
        """world-frame hand pose via FK: (xyz, quat xyzw)."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = self.joints()
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp_pose(self):
        xyz, q = self.hand_pose()
        R = quat_R(*q)
        return xyz + TCP * R[:, 2], q

    # ---------- acting ----------
    def move_joints(self, target, seconds=3.0, tries=4, tol=0.005):
        for i in range(tries):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(ARM)
            pt = JointTrajectoryPoint(positions=[float(x) for x in target])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            err = np.abs(np.array(self.joints()) - np.array(target)).max()
            print(f"  move_joints try{i}: code={code} max_err={err:.4f}", flush=True)
            if err < tol:
                return True
        return err < tol * 3

    def ik(self, xyz_world, quat, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        b = np.array(xyz_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = list(seed or self.joints())
        req.ik_request.timeout.sec = 2
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"  IK failed for {xyz_world} code={None if r is None else r.error_code.val}", flush=True)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_hand(self, xyz_world, quat, seconds=3.0):
        """IK to a world-frame HAND pose, then trajectory. Returns success."""
        q = self.ik(xyz_world, quat)
        if q is None:
            return False
        ok = self.move_joints(q, seconds)
        p, _ = self.hand_pose()
        print(f"  hand now at {p.round(4)} (target {np.round(xyz_world,4)})", flush=True)
        return ok

    def move_tcp(self, xyz_world, quat, seconds=3.0):
        R = quat_R(*quat)
        hand = np.array(xyz_world) - TCP * R[:, 2]
        return self.move_hand(hand, quat, seconds)

    def gripper(self, open_):
        goal = GripperCommand.Goal()
        goal.command.position = float(GRIP["open_m"] if open_ else GRIP["closed_m"])
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        print(f"  gripper {'open' if open_ else 'close'}: reached={r.reached_goal} stalled={r.stalled} fingers={np.round(self.fingers(),4)}", flush=True)
        return r

    def settle(self, seconds=1.0):
        """advance the paused sim by holding the current pose (lets the
        gripper finish moving, objects settle)."""
        self.move_joints(self.joints(), seconds, tries=1)
        return self.fingers()

    def servo(self, dx=0, dy=0, dz=0, ticks=20):
        msg = TwistStamped(); msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(dx), float(dy), float(dz)
        for _ in range(ticks):
            self.twist_pub.publish(msg); rclpy.spin_once(self.node, timeout_sec=0.05)
        stop = TwistStamped(); stop.header.frame_id = TWIST["frame"]
        self.twist_pub.publish(stop)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
