#!/usr/bin/env python3
"""Reusable arm helper: joint state, IK (world-frame TCP poses), trajectories,
FK, gripper. Clients are built once per Arm instance."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = TRAJ["joints"]
# Verified on this machine: /compute_fk with empty frame_id returns the hand
# at the same coordinates TF reports for world->panda_hand, and IK accepts
# world-frame poses. So the planner's model frame IS world here; no offset.
BASE = np.zeros(3)
TCP_OFF = float(M["hand"]["tcp_offset_m"])
_c45 = math.cos(math.pi / 4)
RZ45 = np.array([[_c45, -_c45, 0], [_c45, _c45, 0], [0, 0, 1]])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def hand_R(tilt_deg=0.0, yaw_deg=0.0):
    """Hand pointing down, fingers along world x, flange tilted toward +y by
    tilt_deg (rotation about world x). yaw_deg rotates the hand about world z
    first (180 -> hand +x points to world -y instead of +y)."""
    R0 = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
    y = math.radians(yaw_deg)
    Rz = np.array([[math.cos(y), -math.sin(y), 0],
                   [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    t = math.radians(-tilt_deg)
    Rx = np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)],
                   [0, math.sin(t), math.cos(t)]])
    return Rx @ Rz @ R0


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 10)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj_cli = ActionClient(self.node, FollowJointTrajectory,
                                     TRAJ["port"])
        self.grip_cli = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)
        self.traj_cli.wait_for_server(10)
        self.grip_cli.wait_for_server(10)

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))

    def joints(self, fresh=True):
        if fresh:
            self._js = {}
        t = time.time()
        while not all(j in self._js for j in JOINTS) and time.time() - t < 20:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in JOINTS]

    def fingers(self):
        js = self.joints()
        return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    # ---------- kinematics ----------
    def ik(self, tcp_world, R, seed=None, timeout=60):
        """IK for a TCP position (world) and hand rotation matrix R."""
        hand_world = np.asarray(tcp_world, float) - TCP_OFF * R[:, 2]
        p_base = hand_world - BASE
        # the panda_arm group's tip is panda_link8; panda_hand is link8
        # rotated -45 deg about z (URDF panda_hand_joint), so ask IK for
        # R_link8 = R_hand * Rz(+45 deg)
        q = R_to_quat(R @ RZ45)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        pose = req.ik_request.pose_stamped.pose
        pose.position.x, pose.position.y, pose.position.z = map(float, p_base)
        (pose.orientation.x, pose.orientation.y, pose.orientation.z,
         pose.orientation.w) = map(float, q)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            code = None if res is None else res.error_code.val
            raise RuntimeError(f"IK failed (code {code}) for tcp {tcp_world}")
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def fk(self, q=None, link="panda_hand"):
        """Return (pos_world, R) of link for joint vector q."""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError("FK failed")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        R = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z,
                      p.orientation.w)
        return pos, R

    def tcp(self, q=None):
        pos, R = self.fk(q)
        return pos + TCP_OFF * R[:, 2], R

    # ---------- motion ----------
    def move_joints(self, waypoints, times):
        """waypoints: list of 7-vectors; times: cumulative seconds."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for q, t in zip(waypoints, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.traj_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        gh = send.result()
        if not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q_now = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q_now, waypoints[-1]))
        print(f"traj done error_code={code} max joint err={err:.4f}")
        return code, err

    def move_tcp(self, tcp_world, R, seconds=3.0, seed=None):
        q = self.ik(tcp_world, R, seed)
        return self.move_joints([q], [seconds]), q

    def move_tcp_line(self, p0, p1, R, n=6, seconds=4.0, seed=None):
        """Straight TCP line p0->p1 as IK waypoints in one trajectory."""
        qs = []
        seed = seed if seed is not None else self.arm_q()
        for i in range(1, n + 1):
            p = np.asarray(p0) + (np.asarray(p1) - np.asarray(p0)) * i / n
            seed = self.ik(p, R, seed)
            qs.append(seed)
        times = [seconds * i / n for i in range(1, n + 1)]
        return self.move_joints(qs, times), qs

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f


def wrench(a, n=3):
    """Average external wrench force (panda_link0 frame) over n samples."""
    from geometry_msgs.msg import WrenchStamped
    got = []
    sub = a.node.create_subscription(
        WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
        lambda m: got.append([m.wrench.force.x, m.wrench.force.y, m.wrench.force.z]), 10)
    t = time.time()
    while len(got) < n and time.time() - t < 10:
        rclpy.spin_once(a.node, timeout_sec=0.2)
    a.node.destroy_subscription(sub)
    return np.mean(got, axis=0) if got else None
