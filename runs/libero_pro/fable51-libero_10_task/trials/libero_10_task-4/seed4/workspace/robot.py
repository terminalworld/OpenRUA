#!/usr/bin/env python3
"""Reusable Panda helper: one node, clients built once.
Frames: MoveIt plans in panda_link0; world->panda_link0 = (-0.51,0,0.42).
"""
import math, time
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

# Measured: /compute_fk and /compute_ik on this machine already work in
# WORLD coordinates (FK of current q matches TF world->hand), so no offset.
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])
TCP = 0.1034


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def down_quat(yaw):
    """Hand pointing straight down (+Z hand = -Z world), fingers rotated by yaw
    about world Z. Panda's canonical top-down grasp is q=(1,0,0,0) rotated."""
    # R = Rz(yaw) @ Rx(pi): quaternion product
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    # qz(yaw) = (0,0,sy,cy); qx(pi) = (1,0,0,0); product qz*qx:
    return (cy, sy, 0.0, 0.0)  # (x,y,z,w) -> x=cy, y=sy, z=0, w=0


def ik_down_quat(hand_yaw):
    """Quaternion to REQUEST from IK so that panda_hand ends up pointing down
    with its finger axis at `hand_yaw` (0 = fingers along world y,
    pi/2 = along world x). Measured: IK solves for panda_link8, which is
    rotated -45 deg about z from panda_hand, so request yaw - pi/4."""
    return down_quat(hand_yaw - math.pi / 4)


class Robot:
    def __init__(self):
        m = yaml.safe_load(open("/workspace/machine.yaml"))
        self.m = m
        self.traj = next(a for a in m["actuators"] if a["kind"] == "joint_trajectory")
        self.grip = next(a for a in m["actuators"] if a["kind"] == "gripper")
        self.twist = next(a for a in m["actuators"] if a["kind"] == "cartesian_twist")
        self.joints = self.traj["joints"]
        rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, self.traj["port"])
        self.gc = ActionClient(self.node, GripperCommand, self.grip["port"])
        self.ik = self.node.create_client(GetPositionIK, m["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, self.twist["port"], 10)
        assert self.fjt.wait_for_server(10) and self.gc.wait_for_server(10)
        assert self.ik.wait_for_service(10) and self.fk.wait_for_service(10)
        while not self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, msg):
        self._js.update(zip(msg.name, msg.position))

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joint_state(self, fresh=True):
        if fresh:
            for _ in range(3):
                self.spin(0.1)
        return dict(self._js)

    def arm_q(self):
        js = self.joint_state()
        return [js[j] for j in self.joints]

    def finger_gap(self):
        js = self.joint_state()
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    def _seed(self, q=None):
        s = JointState()
        s.name = list(self.joints)
        s.position = [float(v) for v in (q if q is not None else self.arm_q())]
        return s

    def fk_world(self, q=None):
        """hand pose in world: (xyz, quat xyzw)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        assert r is not None and r.error_code.val == 1, f"FK failed {r}"
        p = r.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik_world(self, xyz, quat, seed=None, at_tcp=True):
        """joint solution for hand (or TCP) at world xyz with quat (xyzw)."""
        xyz = np.asarray(xyz, float)
        if at_tcp:
            R = quat_R(*quat)
            xyz = xyz - TCP * R[:, 2]
        xyz = xyz - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.m["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in self.joints]

    def move_q(self, q, seconds=3.0, waypoints=None, retries=3, tol=0.02):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.joints)
        pts = []
        wps = list(waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = max(abs(a - b) for a, b in zip(self.arm_q(), q))
        # controller lag shows up as -5 with a residual; resend converges
        if err > tol and retries > 0:
            print(f"  [move_q] code={code} residual={err:.3f}, resending")
            return self.move_q(q, seconds, retries=retries - 1, tol=tol)
        return code, err

    def move_world(self, xyz, quat, seconds=3.0, seed=None, at_tcp=True):
        q = self.ik_world(xyz, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            return None
        code, err = self.move_q(q, seconds)
        return q, code, err

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(self.grip["max_effort"])
        fut = self.gc.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        return r.reached_goal, r.stalled, self.finger_gap()

    def servo(self, v, n_ticks, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = self.twist["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n_ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            self.spin(dt)

    def tcp_world(self, q=None):
        xyz, quat = self.fk_world(q)
        R = quat_R(*quat)
        return xyz + TCP * R[:, 2], quat

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()
