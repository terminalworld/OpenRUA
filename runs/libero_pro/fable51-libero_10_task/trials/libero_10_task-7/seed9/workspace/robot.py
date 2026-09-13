#!/usr/bin/env python3
"""Reusable helpers for this Panda: FK, IK, trajectory, gripper, servo, sensing.
Poses are in the WORLD frame (the FK/IK services on this machine answer in world;
panda_link0 sits at world W2B).
"""
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
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
W2B = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 translation (from TF)


def world2base(p):
    return np.asarray(p, float) - W2B


def base2world(p):
    return np.asarray(p, float) + W2B


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):  # (x,y,z,w)
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return np.array([aw*bx + ax*bw + ay*bz - az*by,
                     aw*by - ax*bz + ay*bw + az*bx,
                     aw*bz + ax*by - ay*bx + az*bw,
                     aw*bw - ax*bx - ay*by - az*bz])


def down_quat(yaw):
    """Hand pointing straight down (hand +Z = -world Z), fingers closing along
    a horizontal axis rotated by `yaw` from the base Y axis."""
    rx = np.array([1.0, 0.0, 0.0, 0.0])  # 180 deg about X
    rz = np.array([0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)])
    return quat_mul(rz, rx)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states", lambda m: self._js.__setitem__("m", m), 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
                                      lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---- sensing ----
    def joints(self):
        self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def wrench(self):
        self._wr.pop("m", None)
        while "m" not in self._wr:
            self.spin(0.2)
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])

    def _seed(self, q=None):
        js = JointState()
        js.name = list(ARM)
        js.position = [float(x) for x in (q if q is not None else self.arm_q())]
        return js

    def fk_pose(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed: {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp_pose(self, q=None):
        p, quat = self.fk_pose(q)
        R = quat_R(*quat)
        return p + TCP * R[:, 2], quat

    # ---- IK ----
    def ik_solve(self, pos, quat, seed=None, at_tcp=True, tries=5, max_jump=1.2):
        pos = np.asarray(pos, float)
        if at_tcp:
            pos = pos - TCP * quat_R(*quat)[:, 2]
        seed_q = seed if seed is not None else self.arm_q()
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            req.ik_request.ik_link_name = "panda_hand"
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state = self._seed(seed_q)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                q = [sol[j] for j in ARM]
                jump = np.abs(np.array(q) - np.array(seed_q)).max()
                if jump < max_jump or k == tries - 1:
                    return q
                print(f"  IK attempt {k+1}: branch jump {jump:.2f} rad, retrying", file=sys.stderr)
            else:
                print(f"  IK attempt {k+1} failed: {r and r.error_code.val}", file=sys.stderr)
            seed_q = list(np.array(seed_q) + np.random.uniform(-0.3, 0.3, 7))
        return None

    # ---- motion ----
    def move_q(self, q, seconds=3.0, via=None, retries=3, tol=0.02):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [q]
        for i, w in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(x) for x in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        for attempt in range(retries + 1):
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            print(f"  move_q[{attempt}]: error_code={code} max_joint_err={err:.4f}")
            if err < tol:
                break
            # controller lag: re-send the remaining motion only (single point)
            goal.trajectory.points = [pts[-1]]
            t = max(2.0, seconds * min(1.0, err))
            pts[-1].time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        return code, err

    def move_tcp(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik_solve(pos, quat, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for {pos}")
        code, err = self.move_q(q, seconds)
        p, _ = self.tcp_pose()
        _, qq = self.fk_pose()
        print(f"  tcp now {p.round(4)} quat {qq.round(3)} (target {np.asarray(pos).round(4)} {np.asarray(quat).round(3)})")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        print(f"  gripper: reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}")
        return r

    def servo(self, lin, n=20, ang=(0, 0, 0)):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist.publish(msg)
            self.spin(0.05)
        stop = TwistStamped(); stop.header.frame_id = TW["frame"]
        for _ in range(3):
            self.twist.publish(stop); self.spin(0.05)
