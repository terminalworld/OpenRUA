#!/usr/bin/env python3
"""Reusable robot helpers for this Panda machine (see machine.yaml).

All poses here are in the panda_link0 (arm base) frame unless noted;
world -> base offset is WB (read from TF once at import via tf2_echo).
"""
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
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
# Verified by FK->IK round trip: MoveIt's model root here coincides with
# the world frame (FK output matches camera-derived world coords), so no
# offset is applied.
WB = np.array([0.0, 0.0, 0.0])
TCP = M["hand"]["tcp_offset_m"]


def w2b(p):
    return np.asarray(p, float) - WB


def b2w(p):
    return np.asarray(p, float) + WB


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_mul(a, b):
    """Hamilton product of (x,y,z,w) quaternions: a then b (b in a's frame)."""
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


# The IK tip link is panda_link8; panda_hand = link8 rotated -45 deg about
# Z (verified by FK on both links). Requests for the HAND are converted.
HAND_TO_LINK8 = quat_from_rpy(0.0, 0.0, math.pi / 4)


def down_quat(yaw):
    """Hand Z pointing down (-Z world), fingers' opening axis rotated by yaw
    about world Z. yaw=0 -> hand X along +X base."""
    return quat_from_rpy(math.pi, 0.0, yaw)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js = None
        end = time.time() + 30
        while self._js is None and time.time() < end:
            self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in JOINTS]

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        """Return (pos_base, quat) of link for arm joints q (default current)."""
        if q is None:
            q = self.arm_q()
        self.fk_cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def hand_world(self):
        p, q = self.fk()
        return b2w(p), q

    def tcp_world(self, q=None):
        p, quat = self.fk(q)
        R = quat_R(*quat)
        return b2w(p + TCP * R[:, 2]), quat

    # ---------- IK ----------
    def ik(self, pos_base, quat, seed=None, at_tcp=False, timeout=5.0, attempts=1):
        pos_base = np.asarray(pos_base, float)
        if at_tcp:
            R = quat_R(*quat)
            pos_base = pos_base - TCP * R[:, 2]
        quat = quat_mul(quat, HAND_TO_LINK8)  # hand request -> link8 request
        if seed is None:
            seed = self.arm_q()
        self.ik_cli.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=120)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def ik_world(self, pos_world, quat, **kw):
        return self.ik(w2b(pos_world), quat, **kw)

    # ---------- motion ----------
    def move_q(self, q, seconds=3.0, via=None):
        """Send a trajectory to joint config q (optionally through via points
        [(q, t), ...]). Returns error_code."""
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        for vq, vt in (via or []):
            pt = JointTrajectoryPoint(positions=[float(v) for v in vq])
            pt.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=900)
        r = res.result()
        code = r.result.error_code if r is not None else None
        qa = np.array(self.arm_q())
        err = np.abs(qa - np.array(q)).max()
        print(f"move_q: error_code={code} max_joint_err={err:.4f}")
        return code

    def move_pose_world(self, pos_world, quat, seconds=3.0, at_tcp=False, seed=None):
        q = self.ik_world(pos_world, quat, at_tcp=at_tcp, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for world {pos_world}")
        return self.move_q(q, seconds)

    def gripper(self, width):
        self.grip.wait_for_server(timeout_sec=10)
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}")
        return r

    def twist(self, lin=(0, 0, 0), ang=(0, 0, 0), n=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            self.twist_pub.publish(msg)
            self.spin(dt)
