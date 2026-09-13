#!/usr/bin/env python3
"""Reusable control helpers: one node, clients built once.

World-frame poses; IK via /compute_ik (empty frame_id, verified to be
the same frame as TF `world` on this machine via /compute_fk).
"""
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
DOWN = (1.0, 0.0, 0.0, 0.0)  # hand z down, fingers open along world y


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_down(yaw):
    """Quaternion: hand pointing down, fingers rotated by yaw about world z."""
    # q = Rz(yaw) * Rx(pi)
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    # Rz(yaw) = (0,0,s,c); Rx(pi) = (1,0,0,0); product (w1w2 - v1.v2, ...)
    # (x,y,z,w) = (c*1, s*1*... ) compute explicitly
    w1, x1, y1, z1 = c, 0.0, 0.0, s
    w2, x2, y2, z2 = 0.0, 1.0, 0.0, 0.0
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


class Robot:
    def __init__(self, name="robolib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(timeout_sec=20), "no IK"
        assert self.fk.wait_for_service(timeout_sec=20), "no FK"
        assert self.fjt.wait_for_server(timeout_sec=20), "no FJT"
        assert self.grip.wait_for_server(timeout_sec=20), "no gripper"
        self.spin(0.5)

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))

    def spin(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self, fresh=True):
        if fresh:
            self._js = {}
            while not self._js:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in ARM]

    def finger_gap(self):
        js = self.joints()
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    # ---- kinematics ----
    def hand_pose(self, q=None):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = list(q if q is not None else self.arm_q())
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        assert r is not None and r.error_code.val == 1, f"FK failed {r}"
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def tcp_pose(self, q=None):
        p, quat = self.hand_pose(q)
        R = quat_R(*quat)
        return p + TCP * R[:, 2], quat

    def solve_ik(self, xyz, quat=DOWN, at_tcp=True, seed=None, tries=3):
        xyz = np.array(xyz, dtype=float)
        if at_tcp:
            R = quat_R(*quat)
            xyz = xyz - TCP * R[:, 2]
        seed = list(seed if seed is not None else self.arm_q())
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, xyz)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = seed
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in ARM]
            print(f"  IK try {k+1} failed: {None if r is None else r.error_code.val}")
        return None

    # ---- motion ----
    def move_q(self, q, secs=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via, 1):
                t = secs * i / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        assert h is not None and h.accepted, "FJT goal rejected"
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        res = rf.result()
        code = res.result.error_code if res else None
        self.spin(0.3)
        err = float(np.max(np.abs(np.array(self.arm_q()) - np.array(q))))
        print(f"  move done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_to(self, xyz, quat=DOWN, secs=3.0, at_tcp=True, label="", tol=0.02):
        q = self.solve_ik(xyz, quat, at_tcp)
        if q is None:
            print(f"  !! no IK for {label} {xyz}")
            return False
        code, err = self.move_q(q, secs)
        for _ in range(3):  # resend to converge when tracking lagged
            if err < tol:
                break
            code, err = self.move_q(q, 1.5)
        p, _ = self.tcp_pose()
        print(f"  [{label}] tcp now {p.round(4)} target {np.round(xyz,4)} "
              f"d={np.linalg.norm(p-np.array(xyz)):.4f}")
        return code == 0 and err < 0.02

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        self.spin(0.3)
        print(f"  gripper({width}) reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}")
        return r
