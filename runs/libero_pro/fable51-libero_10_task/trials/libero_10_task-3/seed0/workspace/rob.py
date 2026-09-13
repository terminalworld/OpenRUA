#!/usr/bin/env python3
"""Small control helper for this Panda: joint state, FK/IK (MoveIt), trajectory,
gripper, and cartesian servo. All poses are WORLD frame unless noted; the
planner frame (panda_link0) is world shifted by BASE_T (from /tf)."""
import math
import sys
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
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
# Verified via /compute_fk: MoveIt's model frame here IS `world` (panda_link0
# is reported at (-0.66,0,0.912)), so poses need no base offset.
BASE_T = np.array([0.0, 0.0, 0.0])
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def rot_from_axes(zaxis, xaxis_hint):
    """Rotation whose z column is zaxis and x column is closest to xaxis_hint."""
    z = np.asarray(zaxis, float); z /= np.linalg.norm(z)
    x = np.asarray(xaxis_hint, float); x = x - z * (x @ z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.05): rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None: self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        while self._wr is None: self.spin(0.2)
        f = self._wr.wrench.force; t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    # ---- kinematics (planner frame = panda_link0) ----
    def fk_hand(self, q=None):
        """world pose of panda_hand: (pos, quat, R)."""
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, q))
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_T
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat, quat_to_R(quat)

    def tcp(self, q=None):
        pos, quat, R = self.fk_hand(q)
        return pos + TCP * R[:, 2], quat, R

    def ik_hand(self, pos_w, quat, seed=None, timeout=20.0):
        """IK for panda_hand at world pos/quat -> joint list or None."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        # the group's default tip is panda_link8 (45 deg yaw off the hand)
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pb = np.asarray(pos_w, float) - BASE_T
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = self.arm_q() if seed is None else seed
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(map(float, seed))
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=90)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp(self, tcp_w, R, seed=None, timeout=20.0):
        pos = np.asarray(tcp_w, float) - TCP * R[:, 2]
        return self.ik_hand(pos, R_to_quat(R), seed, timeout)

    # ---- motion ----
    def move_q(self, q, seconds=None, via=None, retries=2, max_rate=0.2):
        """One trajectory to q (optionally through intermediate points `via`).
        Duration defaults to travel/max_rate (rad/s); re-sends on -5/off-target
        (the controller lags long goals; resending converges)."""
        if seconds is None:
            dq = np.abs(np.array(q) - np.array(self.arm_q())).max()
            seconds = max(2.0, dq / max_rate)
        for attempt in range(retries + 1):
            code, err = self._send_traj(q, seconds, via if attempt == 0 else None)
            if code == 0 and err < 0.02:
                return code, err
            print(f"[move_q] retrying ({attempt+1}/{retries})", flush=True)
            seconds = max(2.0, err / max_rate * 1.5)
        return code, err

    def _send_traj(self, q, seconds, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        allq = (via or []) + [q]
        n = len(allq)
        for i, qq in enumerate(allq):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=list(map(float, qq)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = r.result.error_code if r else None
        qa = np.array(self.arm_q())
        err = np.abs(qa - np.array(q)).max()
        print(f"[move_q] code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def ik_tcp_near(self, tcp_w, R, seed=None, max_jump=1.0, tries=6):
        """IK solution whose joints stay within max_jump rad of seed (the IK
        service happily returns far-away branches the controller then cannot
        track and which sweep the hand through the table)."""
        seed = self.arm_q() if seed is None else list(seed)
        best = None
        for i in range(tries):
            s = seed if i == 0 else list(np.array(seed) + np.random.uniform(-0.15, 0.15, len(seed)))
            q = self.ik_tcp(tcp_w, R, s, timeout=5.0)
            if q is None:
                continue
            jump = np.abs(np.array(q) - np.array(seed)).max()
            if best is None or jump < best[0]:
                best = (jump, q)
            if jump <= max_jump:
                return q
        if best is not None:
            print(f"[ik_tcp_near] closest branch still jumps {best[0]:.2f} rad", flush=True)
        return None

    def move_tcp(self, tcp_w, R, seconds=None, seed=None, max_jump=1.0):
        q = self.ik_tcp_near(tcp_w, R, seed, max_jump)
        if q is None:
            print(f"[move_tcp] IK FAILED (or too far) for {np.round(tcp_w,3)}", flush=True)
            return None
        code, err = self.move_q(q, seconds)
        p, _, Rn = self.tcp()
        print(f"[move_tcp] tcp now {np.round(p,4)} z-axis {np.round(Rn[:,2],3)} (target {np.round(tcp_w,4)})", flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.finger()
        print(f"[gripper] target={width} reached={r.reached_goal} stalled={r.stalled} fingers={np.round(f,4)}", flush=True)
        return f

    def servo(self, lin, ang=(0, 0, 0), ticks=20):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)


if __name__ == "__main__":
    r = Robot()
    q = r.arm_q()
    print("q:", np.round(q, 4))
    print("fingers:", r.finger())
    p, quat, R = r.fk_hand(q)
    print("hand world:", np.round(p, 4), "quat:", np.round(quat, 4))
    print("R:\n", np.round(R, 3))
    print("tcp world:", np.round(p + TCP * R[:, 2], 4))
    print("wrench:", r.wrench())
