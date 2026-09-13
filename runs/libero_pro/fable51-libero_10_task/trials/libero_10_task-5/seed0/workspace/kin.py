#!/usr/bin/env python3
"""Shared kinematics helpers: one node, reusable FK/IK/joint-state/trajectory/gripper clients."""
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP_OFF = 0.1034


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(q1, q2):
    """Hamilton product, xyzw convention: q1 then q2 (q2 in q1's local frame)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


class Robot:
    def __init__(self, name="kin"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.fk_cli.wait_for_service(10); self.ik_cli.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)

    def _on_js(self, m):
        self._js["m"] = m

    def _on_wr(self, m):
        self._wr["m"] = m

    def spin(self, t=0.3):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self, fresh=True):
        if fresh:
            self._js.clear()
            while "m" not in self._js:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[a] for a in ARM]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def wrench(self):
        self._wr.clear()
        end = time.time() + 3
        while "m" not in self._wr and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if "m" not in self._wr:
            return None
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])

    def fk(self, q, link="panda_hand"):
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, q))
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def tcp(self, q=None):
        q = self.arm_q() if q is None else q
        pos, quat = self.fk(q)
        R = quat_R(*quat)
        return pos + TCP_OFF * R[:, 2], quat, R

    def ik(self, pos, quat, seed=None, at_tcp=True, timeout=2.0):
        """pos/quat: world-frame pose of TCP (at_tcp) or hand. Returns joint list or None."""
        pos = np.asarray(pos, float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP_OFF * R[:, 2]
        # machine quirk: /compute_ik solves for panda_link8 (hand yawed -45deg
        # about its z); pre-rotate the request by +45deg about local z so the
        # HAND ends up at the requested orientation (verified via /compute_fk)
        quat = quat_mul(quat, np.array([0, 0, np.sin(np.pi / 8), np.cos(np.pi / 8)]))
        seed = self.arm_q() if seed is None else seed
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = "panda_arm"
        r.pose_stamped.header.frame_id = ""
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = map(float, pos)
        r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y, r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w = map(float, quat)
        r.robot_state.joint_state.name = ARM
        r.robot_state.joint_state.position = list(map(float, seed))
        r.timeout.sec = int(timeout); r.timeout.nanosec = int((timeout % 1) * 1e9)
        r.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[a] for a in ARM]

    def move(self, q, seconds=3.0, retries=2, via=None):
        """Send trajectory (optionally through via points: list of (q, t)). Returns error code."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        # controller tracks only ~0.2 rad/s reliably: stretch duration for long moves
        dmax = float(np.abs(np.array(self.arm_q()) - np.array(q)).max())
        need = dmax / 0.12 + 0.5
        if need > seconds:
            print(f"  move: stretching duration {seconds:.1f}s -> {need:.1f}s (dq={dmax:.2f})")
            seconds = need
        if via:
            for vq, vt in via:
                p = JointTrajectoryPoint(positions=list(map(float, vq)))
                p.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
                pts.append(p)
        p = JointTrajectoryPoint(positions=list(map(float, q)))
        p.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(p)
        goal.trajectory.points = pts
        code = None
        for attempt in range(retries + 1):
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            print(f"  move attempt {attempt}: code={code} max joint err={err:.4f}")
            if code == 0 or err < 0.01:
                break
        return code

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def report(self, tag=""):
        pos, quat, R = self.tcp()
        f = self.fingers()
        print(f"[{tag}] tcp={pos.round(4)} quat={quat.round(4)} zaxis={R[:,2].round(3)} xaxis={R[:,0].round(3)} fingers={f[0]:.4f},{f[1]:.4f}")
        return pos
