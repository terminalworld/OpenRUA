#!/usr/bin/env python3
"""Small helper library for this Panda: joint state, FK, IK, trajectory,
gripper, and world<->base frame conversion. Poses handed to IK/FK are in
the planner model frame (panda_link0); world_to_base/base_to_world convert.
"""
import sys
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
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
TCP = M["hand"]["tcp_offset_m"]
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # FK of panda_link0 returns (-0.51,0,0.42):
# the planner model frame IS the world frame on this machine (docs say otherwise)


def world_to_base(p):
    return np.asarray(p, float) - BASE_IN_WORLD


def base_to_world(p):
    return np.asarray(p, float) + BASE_IN_WORLD


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
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def down_quat(yaw=0.0):
    """Hand pointing straight down (hand +Z = world -Z); fingers close
    along the hand Y axis, rotated by `yaw` about world Z. yaw=0 -> hand
    Y along world Y (the home-configuration orientation)."""
    # hand X -> world X, hand Y -> world -Y, hand Z -> world -Z (a 180deg
    # flip about X), then yaw about world Z.
    Rflip = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return R_to_quat(Rz @ Rflip)


class Arm:
    def __init__(self, name="arm_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 30:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return np.array([j[n] for n in JOINTS])

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        """Return (pos, quat) of link in the base frame."""
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
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp_world(self, q=None):
        """World position of the fingertip point (TCP) and the hand quat."""
        p, quat = self.fk(q)
        R = quat_to_R(quat)
        return base_to_world(p + TCP * R[:, 2]), quat

    # ---------- planning ----------
    def ik(self, pos_base, quat, seed=None, at_tcp=True, timeout=60):
        """IK for the hand (or TCP if at_tcp) at pos_base (base frame)."""
        pos = np.asarray(pos_base, float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(quat)[:, 2]
        # IK tip link is panda_link8, which is panda_hand rotated -45deg about
        # its Z (tf_static). Convert the requested HAND quat to a link8 quat.
        c, s_ = np.cos(np.pi / 4), np.sin(np.pi / 4)
        Rz45 = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
        quat = R_to_quat(quat_to_R(quat) @ Rz45)
        if seed is None:
            seed = self.arm_q()
        self.ik_cli.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timed out")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in JOINTS])

    def ik_world(self, pos_world, quat, **kw):
        return self.ik(world_to_base(pos_world), quat, **kw)

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, via=None):
        """Execute a trajectory to q (optionally through `via` list of
        (q, t) waypoints). Returns error_code."""
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        for qq, t in (via or []):
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(self.arm_q() - np.asarray(q))
        print(f"  traj error_code={code} max joint err={err.max():.4f}", flush=True)
        return code

    def move_tcp_world(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik_world(pos_world, quat, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for TCP world {pos_world}")
        code = self.move_q(q, seconds)
        tcp, _ = self.tcp_world()
        print(f"  TCP world now {tcp.round(4)} (target {np.round(pos_world, 4)})", flush=True)
        return q

    def gripper(self, width):
        self.grip.wait_for_server(timeout_sec=10)
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap

    def servo(self, v_world, ticks=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v_world)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=dt)
        stop = TwistStamped()
        stop.header.frame_id = TW["frame"]
        for _ in range(3):
            self.tw_pub.publish(stop)
            rclpy.spin_once(self.node, timeout_sec=dt)


if __name__ == "__main__":
    a = Arm()
    q = a.arm_q()
    print("q =", q.round(4))
    p, quat = a.fk(q)
    print("hand base pos", p.round(4), "quat", quat.round(4))
    tcp, _ = a.tcp_world()
    print("TCP world", tcp.round(4))
    print("down_quat(0) =", down_quat(0).round(4))
    print("finger gap", a.finger_gap())
