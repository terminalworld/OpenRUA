#!/usr/bin/env python3
"""Reusable robot helper: one node, persistent clients.

    from rob import Robot
    r = Robot()
    r.joints()            # dict name->pos
    r.fk()                # (xyz, quat xyzw) of panda_hand in world
    r.ik(xyz, quat)       # arm joint list or None (hand frame, world coords)
    r.move_joints(list, secs)
    r.move_pose(xyz, quat, secs, tcp=False)
    r.gripper(width)      # per-finger position
    r.servo(vx,vy,vz, wx,wy,wz, n)  # twist bursts
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE = np.array([0.0, 0.0, 0.0])  # FK/IK on this machine are already in world (verified vs TF)


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_z(theta):
    return np.array([0, 0, np.sin(theta / 2), np.cos(theta / 2)])


def down_quat(yaw):
    """Hand pointing straight down (hand +Z = world -Z); fingers close
    along the world axis obtained by rotating world Y by `yaw`."""
    return quat_mul(quat_z(yaw), np.array([1.0, 0.0, 0.0, 0.0]))


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
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)
        while self._js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.3):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 5
        while self._wr is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self._wr is None:
            return None
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    # ---- kinematics (world frame; planner works in panda_link0) ----
    def fk(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.header.frame_id = ""
        js = JointState()
        js.name = list(ARM)
        js.position = list(q if q is not None else self.arm_q())
        req.robot_state.joint_state = js
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print("FK failed", None if res is None else res.error_code.val)
            return None
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q_ = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return xyz, q_

    def ik(self, xyz, quat, seed=None, tcp=False, tries=3):
        xyz = np.array(xyz, dtype=float)
        if tcp:
            xyz = xyz - TCP * quat_R(quat)[:, 2]
        local = xyz - BASE
        # IK tip link is panda_link8 (verified): convert the requested
        # panda_hand orientation (hand = link8 rotated -45deg about z)
        quat = quat_mul(np.array(quat, dtype=float), quat_z(np.pi / 4))
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, local)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        js = JointState()
        js.name = list(ARM)
        js.position = list(seed if seed is not None else self.arm_q())
        req.ik_request.robot_state.joint_state = js
        for _ in range(tries):
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[n] for n in ARM]
            print("IK fail", None if res is None else res.error_code.val)
        return None

    # ---- motion ----
    def move_joints(self, q, secs=3.0, retries=3, tol=0.01):
        for i in range(retries):
            code, err = self._move_once(q, secs)
            if err < tol:
                break
            print(f"  retry {i + 1}: err {err:.3f}")
        return code, err

    def _move_once(self, q, secs=3.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"move_joints code={code} max_err={err:.4f}")
        return code, err

    def move_pose(self, xyz, quat, secs=3.0, tcp=False, seed=None):
        q = self.ik(xyz, quat, seed=seed, tcp=tcp)
        if q is None:
            print("move_pose: no IK for", xyz)
            return None
        self.move_joints(q, secs)
        got = self.fk()
        print("  hand now", np.round(got[0], 4), "tcp", np.round(got[0] + TCP * quat_R(got[1])[:, 2], 4))
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
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}")
        return r

    def servo(self, lin, ang=(0, 0, 0), n=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=dt)

    def tcp(self):
        xyz, q = self.fk()
        return xyz + TCP * quat_R(q)[:, 2], q
