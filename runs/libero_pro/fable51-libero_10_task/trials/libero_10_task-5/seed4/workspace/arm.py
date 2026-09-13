#!/usr/bin/env python3
"""Small helper library: joint state, FK, IK, trajectory, gripper.

World <-> base: panda_link0 sits at WORLD_BASE in world (from TF).
All pose helpers take WORLD-frame TCP poses (fingertip point) and
convert to the planner's base frame internally.
"""
import sys
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from geometry_msgs.msg import WrenchStamped

# Verified: /compute_fk for the current config returns x=-0.293 which matches
# the hand seen in birdview at world x=-0.294, so the planner's model frame
# IS world on this machine (panda_link0 is at (-0.75,0,0.912) in it).
WORLD_BASE = np.array([0.0, 0.0, 0.0])
TCP_OFF = 0.1034
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
FJT = "/panda_arm_controller/follow_joint_trajectory"
GRIP = "/franka_gripper/gripper_action"


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    """rotation matrix -> quaternion (x,y,z,w)"""
    m = R
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# hand pointing down (hand z = -Z), fingers along world X (hand y = +X)
R_DOWN_FX = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
# hand pointing down, fingers along world Y (hand y = -Y, hand x = +X)
R_DOWN_FY = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)


class Arm:
    def __init__(self, name="arm_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 10)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT)
        self.grip = ActionClient(self.node, GripperCommand, GRIP)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        if fresh:
            self._js = None
        for _ in range(100):
            if self._js is not None:
                break
            self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in JOINTS])

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        for _ in range(50):
            if self._wr is not None:
                break
            self.spin(0.2)
        if self._wr is None:
            return None
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    # ---------------- kinematics ----------------
    def fk(self, q=None, link="panda_hand"):
        """returns (pos_world, R) of link for arm config q (default: current)"""
        if q is None:
            q = self.arm_q()
        if not self.fk_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no FK service")
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + WORLD_BASE
        R = quat_R([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, R

    def tcp(self, q=None):
        pos, R = self.fk(q)
        return pos + TCP_OFF * R[:, 2], R

    def ik(self, tcp_world, R, seed=None, attempts=3):
        """IK for a TCP pose in world. Returns joint array or None."""
        hand = np.asarray(tcp_world, float) - TCP_OFF * R[:, 2]
        base = hand - WORLD_BASE
        # the IK tip link is panda_link8, which is panda_hand rotated by
        # +45deg about z (panda_hand_joint rpy z=-0.785). Verified: asking
        # for hand y=-X returned a config whose FK hand y was 45deg off.
        q = R_quat(R @ Rz(np.pi / 4))
        if seed is None:
            seed = self.arm_q()
        if not self.ik_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no IK service")
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "panda_arm"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, base)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
            req.ik_request.robot_state.joint_state.name = JOINTS
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return np.array([sol[j] for j in JOINTS])
        return None

    # ---------------- acting ----------------
    def move_q(self, q, seconds=3.0, via=None):
        """one trajectory goal through optional via points to q"""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pts = []
        wps = (via or []) + [q]
        for i, wp in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        qn = self.arm_q()
        err = np.abs(qn - np.asarray(q)).max()
        print(f"move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, tcp_world, R, seconds=3.0, seed=None, max_jump=None):
        q = self.ik(tcp_world, R, seed=seed)
        if q is None:
            print("IK FAILED for", np.round(tcp_world, 3))
            return None
        q0 = self.arm_q()
        jump = np.abs(q - q0).max()
        print(f"IK ok, max joint jump {jump:.3f} rad")
        if max_jump is not None and jump > max_jump:
            print("jump too large; not moving")
            return None
        self.move_q(q, seconds)
        p, Rn = self.tcp()
        print("tcp now", np.round(p, 4), "target", np.round(tcp_world, 4),
              "err", np.round(np.linalg.norm(p - tcp_world), 4))
        return q

    def gripper(self, width, timeout=300):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"gripper: reached={r.reached_goal} stalled={r.stalled} fingers={gap}")
        return gap
