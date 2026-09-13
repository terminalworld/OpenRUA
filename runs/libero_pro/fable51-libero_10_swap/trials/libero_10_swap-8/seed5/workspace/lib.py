"""Shared robot helpers: FK/IK, trajectory, gripper, joint state, servo.

World frame → panda_link0 offset comes from TF (world→panda_link0).
IK/FK are done in panda_link0 (leave frame_id empty per machine.yaml).
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
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
# Verified empirically: /compute_fk and /compute_ik on this machine work in
# the WORLD frame (FK of panda_link0 returns (-0.66, 0, 0.912)), so no offset.
BASE_IN_WORLD = np.zeros(3)
TCP = M["hand"]["tcp_offset_m"]


def down_quat(yaw_deg=0.0):
    """Hand z pointing world -z; yaw_deg rotates about world z.
    yaw=0: fingers open along world y; yaw=90: fingers along world x."""
    r = Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])
    return r.as_quat()  # x,y,z,w


class Robot:
    def __init__(self, name="lib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 10:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def fk_world(self, q=None, link="panda_hand"):
        """Return (pos_world, quat) of link for joint config q (default current)."""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def tcp_world(self, q=None):
        pos, quat = self.fk_world(q)
        R = Rot.from_quat(quat).as_matrix()
        return pos + TCP * R[:, 2], quat

    # ---------------- planning ----------------
    def ik_world(self, pos, quat, at_tcp=True, seed=None, tries=3):
        """IK for a world pose of the TCP (or hand). Returns joint list or None."""
        pos = np.array(pos, dtype=float)
        if at_tcp:
            R = Rot.from_quat(quat).as_matrix()
            pos = pos - TCP * R[:, 2]
        pos = pos - BASE_IN_WORLD
        # The IK group's tip link is panda_link8 (verified via FK); panda_hand
        # = link8 rotated -45 deg about z.  Convert the hand quaternion.
        quat = (Rot.from_quat(quat) * Rot.from_euler("z", 45, degrees=True)).as_quat()
        seed = seed if seed is not None else self.arm_q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 1
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    # ---------------- acting ----------------
    def move_q(self, q, seconds=3.0, via=None, retries=3, tol=0.02):
        """Send a trajectory and wait; re-send until within tol (this
        machine's joint7 tops out near 0.2 rad/s, so long yaw moves need
        several passes / long durations)."""
        cur = np.array(self.arm_q())
        delta = np.abs(cur - np.array(q)).max()
        seconds = max(seconds, delta / 0.18)
        code, err = self._send_traj(q, seconds, via)
        for _ in range(retries):
            if err <= tol:
                break
            code, err = self._send_traj(q, max(2.0, err / 0.15), None)
        return code, err

    def _send_traj(self, q, seconds, via):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [q]
        n = len(wps)
        for i, wp in enumerate(wps):
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            t = seconds * (i + 1) / n
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        cur = np.array(self.arm_q())
        err = np.abs(cur - np.array(q)).max()
        print(f"  move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik_world(pos, quat, seed=seed)
        if q is None:
            print(f"  IK FAILED for {np.round(pos,3)}")
            return None
        self.move_q(q, seconds)
        p, _ = self.tcp_world()
        print(f"  tcp now {np.round(p,4)} (target {np.round(pos,4)}) err={np.linalg.norm(p-pos):.4f}")
        return q

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap

    def servo(self, v, n=20, frame=None):
        """Stream n twist messages (linear m/s in base frame)."""
        msg = TwistStamped()
        msg.header.frame_id = frame or TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
