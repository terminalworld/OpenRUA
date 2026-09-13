#!/usr/bin/env python3
"""Helper library: IK / FK / trajectory / gripper / servo for this Panda.

World frame <-> base frame: base (panda_link0) sits at world (-0.75, 0, 0.912).
All public functions take WORLD coordinates.
"""
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# Verified: /compute_ik and /compute_fk on this machine use WORLD coordinates
# (FK of panda_link0 returns (-0.75, 0, 0.912)); no base offset needed.
BASE_W = np.array([0.0, 0.0, 0.0])
TCP = 0.1034

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)

    def _on_js(self, msg):
        self._js["m"] = msg

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    # ---- kinematics -------------------------------------------------
    @staticmethod
    def quat_down(yaw_deg):
        """Hand pointing straight down, fingers closing along direction
        (sin yaw, -cos yaw) in world; yaw=0 -> fingers close along world y."""
        return (R.from_euler("z", yaw_deg, degrees=True)
                * R.from_euler("x", 180, degrees=True)).as_quat()

    def hand_pose_world(self):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        seed = JointState()
        for n, p in zip(JOINTS, self.arm_q()):
            seed.name.append(n); seed.position.append(p)
        req.robot_state.joint_state = seed
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_W
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q

    def tcp_world(self):
        pos, q = self.hand_pose_world()
        return pos + R.from_quat(q).apply([0, 0, TCP]), q

    def solve_ik(self, tcp_w, quat, seed_q=None):
        """tcp_w: fingertip-centre target in world. Returns joint list or None."""
        hand_w = np.asarray(tcp_w, float) - R.from_quat(quat).apply([0, 0, TCP])
        hand_b = hand_w - BASE_W
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        seed = JointState()
        for n, v in zip(JOINTS, seed_q or self.arm_q()):
            seed.name.append(n); seed.position.append(float(v))
        req.ik_request.robot_state.joint_state = seed
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK failed code={None if res is None else res.error_code.val}", flush=True)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    # ---- motion -----------------------------------------------------
    def move_joints(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pts = []
        if via:
            n = len(via) + 1
            for i, vq in enumerate(via, 1):
                t = seconds * i / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in vq])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"traj done code={code} max_joint_err={err:.4f}", flush=True)
        return code

    def move_tcp(self, tcp_w, quat, seconds=3.0, seed_q=None):
        q = self.solve_ik(tcp_w, quat, seed_q)
        if q is None:
            return None
        self.move_joints(q, seconds)
        pos, _ = self.tcp_world()
        print(f"tcp now {pos.round(4)} target {np.round(tcp_w,4)}", flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={self.finger_gap()}", flush=True)
        return r

    def servo(self, vx=0.0, vy=0.0, vz=0.0, ticks=20):
        msg = TwistStamped(); msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        for _ in range(ticks):
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def close(self):
        self.node.destroy_node(); rclpy.shutdown()
