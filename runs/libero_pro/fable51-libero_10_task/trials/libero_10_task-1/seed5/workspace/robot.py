#!/usr/bin/env python3
"""Reusable robot helpers: joints, FK, IK, trajectory, gripper (clients built once)."""
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


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(20); self.grip.wait_for_server(20)
        self.ik.wait_for_service(20); self.fk.wait_for_service(20)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def fk_hand(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(q if q is not None else self.arm_q())
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z]), \
            np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def tcp_world(self, q=None):
        p, quat = self.fk_hand(q)
        R = quat_R(*quat)
        return p + TCP * R[:, 2], quat

    def solve_ik(self, pos, quat, seed=None, at_tcp=True, timeout=60):
        pos = np.array(pos, float)
        quat = [float(v) for v in quat]
        if at_tcp:
            pos = pos - TCP * quat_R(*quat)[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = [float(v) for v in pos]
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(seed if seed is not None else self.arm_q())
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None, (None if r is None else r.error_code.val)
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[n] for n in ARM], 1

    def move_joints(self, q, seconds=3.0, timeout=600, tol=0.01, tries=4):
        """Send the goal; the controller often stops short (code -5) on the
        first pass, so re-send the same goal until the joints converge."""
        for k in range(tries):
            code, err = self._send_traj(q, seconds, timeout)
            if err < tol:
                return code, err
            print(f"    retry {k}: code={code} jerr={err:.4f}", flush=True)
            seconds = max(1.5, seconds * 0.6)
        return code, err

    def _send_traj(self, q, seconds, timeout):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result()
        code = r.result.error_code if r else None
        err = float(np.max(np.abs(np.array(self.arm_q()) - np.array(q))))
        return code, err

    def move_tcp(self, pos, quat, seconds=3.0, seed=None):
        q, code = self.solve_ik(pos, quat, seed=seed)
        if q is None:
            print(f"  IK FAILED code={code} for {np.round(pos,3)}", flush=True)
            return None
        delta = float(np.max(np.abs(np.array(q) - np.array(self.arm_q()))))
        seconds = max(seconds, delta / 0.5)
        c, err = self.move_joints(q, seconds)
        tcp, _ = self.tcp_world()
        print(f"  move -> traj code={c} jerr={err:.4f} tcp_now={np.round(tcp,4)} target={np.round(pos,4)}", flush=True)
        return tcp

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result()
        f = self.fingers()
        print(f"  gripper({width}) reached={r.result.reached_goal if r else None} stalled={r.result.stalled if r else None} fingers={f}", flush=True)
        return f
