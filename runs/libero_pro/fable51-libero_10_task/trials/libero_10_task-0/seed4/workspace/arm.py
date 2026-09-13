#!/usr/bin/env python3
"""Arm control helpers for this Panda: IK -> trajectory, gripper, FK.

World<->base: base (panda_link0) sits at world (-0.51, 0, 0.42), no rotation.
TCP is hand-frame +Z * tcp_offset. Downward hand with fingers along world Y
is quaternion (1,0,0,0); yaw rotates that about world Z.
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
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_IN_WORLD = np.zeros(3)  # MoveIt model frame == world on this machine (verified via FK)


def quat_down(yaw=0.0):
    """Quaternion (x,y,z,w) for hand pointing down, fingers along world Y
    rotated by yaw about world Z."""
    # R = Rz(yaw) @ Rx(pi); q = qz * qx ; qx = (1,0,0,0), qz = (0,0,s,c)
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    # (0,0,s,c) * (1,0,0,0):
    # w = c*0 - 0 = 0? do full Hamilton product:
    w1, x1, y1, z1 = c, 0.0, 0.0, s
    w2, x2, y2, z2 = 0.0, 1.0, 0.0, 0.0
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


class Arm:
    def __init__(self):
        M = yaml.safe_load(open("/workspace/machine.yaml"))
        self.traj = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
        self.grip = next(a for a in M["actuators"] if a["kind"] == "gripper")
        self.joints = self.traj["joints"]
        self.tcp_off = float(M["hand"]["tcp_offset_m"])
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_ctl")
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, self.traj["port"])
        self.gc = ActionClient(self.node, GripperCommand, self.grip["port"])
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10), "no FJT server"
        assert self.gc.wait_for_server(10), "no gripper server"
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fk.wait_for_service(10), "no FK"

    def _on_js(self, m):
        self._js["m"] = m

    # ---------- sensing ----------
    def joint_state(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 20:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        js = self.joint_state()
        return [js[j] for j in self.joints]

    def finger_gap(self):
        js = self.joint_state()
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    def wrench(self):
        self._wr.pop("m", None)
        t0 = time.time()
        while "m" not in self._wr and time.time() - t0 < 5:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if "m" not in self._wr:
            return None
        f = self._wr["m"].wrench.force
        return (f.x, f.y, f.z)

    def hand_pose_world(self, q=None):
        """FK of panda_hand -> (xyz in world, quat)."""
        q = q or self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(self.joints)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp_world(self):
        xyz, q = self.hand_pose_world()
        x, y, z, w = q
        # hand z axis in world = third column of R(q)
        zax = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
        return xyz + self.tcp_off * zax

    # ---------- IK ----------
    def ik_hand(self, hand_xyz_world, quat, seed=None, attempts=3):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        b = np.array(hand_xyz_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        seed = seed or self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(self.joints)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        for _ in range(attempts):
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in self.joints]
        raise RuntimeError(f"IK failed: {None if res is None else res.error_code.val}")

    def ik_tcp(self, tcp_xyz_world, yaw=0.0, seed=None):
        quat = quat_down(yaw)
        x, y, z, w = quat
        zax = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
        hand = np.array(tcp_xyz_world) - self.tcp_off * zax
        return self.ik_hand(hand, quat, seed)

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joints)
        pts = []
        if via:
            n = len(via) + 1
            for i, vq in enumerate(via, 1):
                pt = JointTrajectoryPoint(positions=[float(v) for v in vq])
                t = seconds * i / n
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_tcp(self, tcp_xyz_world, yaw=0.0, seconds=3.0):
        q = self.ik_tcp(tcp_xyz_world, yaw)
        code, err = self.move_q(q, seconds)
        tcp = self.tcp_world()
        print(f"  tcp now {tcp.round(4)} target {np.round(tcp_xyz_world, 4)} "
              f"d={np.linalg.norm(tcp - tcp_xyz_world):.4f}", flush=True)
        return tcp

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(self.grip.get("max_effort", 30.0))
        fut = self.gc.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} "
              f"gap={gap:.4f}", flush=True)
        return gap

    def open(self):
        return self.gripper(self.grip["open_m"])

    def close(self):
        return self.gripper(self.grip["closed_m"])
