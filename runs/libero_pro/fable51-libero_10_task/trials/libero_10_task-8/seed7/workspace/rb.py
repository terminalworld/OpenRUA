#!/usr/bin/env python3
"""Small robot helper library: joint state, FK, IK, trajectory, gripper."""
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

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
# FK/IK services on this machine answer in the WORLD frame (verified:
# FK of the current config matches tf world->panda_hand), so no offset.
BASE_IN_WORLD = np.zeros(3)
TCP = M["hand"]["tcp_offset_m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def down_quat(yaw):
    """Hand pointing straight down (hand +Z = world -Z), fingers closing
    along an axis rotated by `yaw` about world Z (yaw=0: fingers along
    world Y for the Panda hand convention, roughly)."""
    # Rotation: 180 deg about X, then yaw about Z
    # q = qz(yaw) * qx(pi)
    cx, sx = math.cos(math.pi / 2), math.sin(math.pi / 2)
    qx = np.array([sx, 0, 0, cx])
    cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    qz = np.array([0, 0, sz, cz])
    return quat_mul(qz, qx)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


class Robot:
    def __init__(self, name="rb"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def wrench(self):
        self._wr = None
        while self._wr is None:
            self.spin(0.2)
        f = self._wr.wrench.force
        t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def _seed(self, q=None):
        js = JointState()
        q = q if q is not None else self.arm_q()
        js.name = list(ARM)
        js.position = [float(v) for v in q]
        return js

    def fk_hand(self, q=None, link="panda_hand"):
        """Hand pose in the planning (base) frame -> returned in WORLD."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def ik_hand(self, pos_world, quat, seed=None, timeout=60):
        """IK for the hand frame at a WORLD pose. Returns joint list or None."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        # default tip is panda_link8 (45 deg yaw off the hand); be explicit
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pb = np.array(pos_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp(self, tcp_world, quat, **kw):
        R = quat_R(*quat)
        hand = np.array(tcp_world) - TCP * R[:, 2]
        return self.ik_hand(hand, quat, **kw)

    def tcp_pose(self, q=None):
        pos, quat = self.fk_hand(q)
        R = quat_R(*quat)
        return pos + TCP * R[:, 2], quat

    def move(self, q, seconds=3.0, waypoints=None, retries=2):
        """Send a trajectory to joint config q (optionally via waypoints list of (q, t))."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if waypoints:
            for wq, wt in waypoints:
                pt = JointTrajectoryPoint(positions=[float(v) for v in wq])
                pt.time_from_start = Duration(sec=int(wt), nanosec=int((wt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        qa = np.array(self.arm_q())
        err = np.abs(qa - np.array(q)).max()
        print(f"move done code={code} max_joint_err={err:.4f}", flush=True)
        # controller lag on long goals: resend the (short) remainder
        if err > 0.01 and retries > 0:
            return self.move(q, max(1.5, seconds / 2), retries=retries - 1)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f
