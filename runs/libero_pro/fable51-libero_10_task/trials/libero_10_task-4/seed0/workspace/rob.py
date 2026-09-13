#!/usr/bin/env python3
"""Helper library for this Panda workstation: FK, IK, trajectories, gripper,
joint state, world<->base conversion. Import from scripts in /workspace."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
# world -> panda_link0 (measured via tf2_echo): pure translation
W2B_T = np.array([-0.510, 0.0, 0.420])


def world_to_base(p):
    return np.asarray(p, float) - W2B_T


def base_to_world(p):
    return np.asarray(p, float) + W2B_T


# ---------------- kinematics ----------------
def _rot(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rot(rpy)
    T[:3, 3] = xyz
    return T


def _rz(q):
    T = np.eye(4)
    T[:3, :3] = _rot((0, 0, q))
    return T


# joint origins from URDF (parent->child), axis z for all revolute joints
_ORIGINS = [
    ((0, 0, 0.333), (0, 0, 0)),
    ((0, 0, 0), (-math.pi / 2, 0, 0)),
    ((0, -0.316, 0), (math.pi / 2, 0, 0)),
    ((0.0825, 0, 0), (math.pi / 2, 0, 0)),
    ((-0.0825, 0.384, 0), (-math.pi / 2, 0, 0)),
    ((0, 0, 0), (math.pi / 2, 0, 0)),
    ((0.088, 0, 0), (math.pi / 2, 0, 0)),
]
_T8 = _T((0, 0, 0.107), (0, 0, 0))
_THAND = _T((0, 0, 0), (0, 0, -math.pi / 4))


def fk_hand(q):
    """4x4 pose of panda_hand in panda_link0 frame."""
    T = np.eye(4)
    for (xyz, rpy), qi in zip(_ORIGINS, q):
        T = T @ _T(xyz, rpy) @ _rz(qi)
    return T @ _T8 @ _THAND


def fk_tcp(q):
    T = fk_hand(q)
    T = T.copy()
    T[:3, 3] += TCP_OFF * T[:3, 2]
    return T


def quat_from_R(R):
    """(x,y,z,w) from rotation matrix."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


def R_from_quat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def top_down_R(yaw):
    """Hand rotation: z axis pointing down (-Z base), hand x axis rotated by
    yaw about base z. Fingers open along hand y."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])  # Rz(yaw) @ Rx(pi)


# ---------------- ROS client ----------------
class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
        end = time.time() + 20
        while self._js is None and time.time() < end:
            self.spin(0.1)
        if self._js is None:
            raise RuntimeError("no /joint_states")
        d = dict(zip(self._js.name, self._js.position))
        return d

    def q(self, fresh=True):
        d = self.joints(fresh)
        return np.array([d[j] for j in JOINTS])

    def fingers(self, fresh=True):
        d = self.joints(fresh)
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def tcp_world(self, q=None):
        if q is None:
            q = self.q()
        T = fk_tcp(q)
        return base_to_world(T[:3, 3])

    # ---- IK ----
    def ik_solve(self, pos_base, quat_xyzw, seed=None, timeout=60.0, attempts=1):
        if not self.ik.wait_for_service(timeout_sec=10):
            raise RuntimeError("IK service unavailable")
        if seed is None:
            seed = self.q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat_xyzw)
        js = JointState()
        js.name = list(JOINTS)
        js.position = [float(v) for v in seed]
        req.ik_request.robot_state.joint_state = js
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1
        for _ in range(attempts):
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
            res = fut.result()
            if res is None:
                raise RuntimeError("IK timeout")
            if res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return np.array([sol[j] for j in JOINTS])
        return None

    def ik_tcp(self, tcp_world_pos, R, seed=None, **kw):
        """IK for a TCP position (world frame) with hand rotation R.
        Machine fact (measured): /compute_ik takes the pose in the WORLD
        frame and solves for panda_link8 (= panda_hand rotated +45deg about z)."""
        R = np.asarray(R, float)
        hand_world = np.asarray(tcp_world_pos, float) - TCP_OFF * R[:, 2]
        R8 = R @ _rot((0, 0, math.pi / 4))
        return self.ik_solve(hand_world, quat_from_R(R8), seed=seed, **kw)

    # ---- motion ----
    def move_traj(self, points, times, wait=True):
        """points: list of joint vectors; times: cumulative seconds."""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for pt, t in zip(points, times):
            jp = JointTrajectoryPoint(positions=[float(v) for v in pt])
            jp.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(jp)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        r = res.result()
        code = r.result.error_code if r else None
        qf = self.q()
        err = np.abs(qf - np.asarray(points[-1])).max()
        print(f"traj done code={code} max_joint_err={err:.4f}")
        return code, qf

    def move_to(self, qtarget, seconds=3.0):
        return self.move_traj([qtarget], [seconds])

    def gripper(self, width, timeout=120):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        gh = fut.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result
        f = self.fingers()
        print(f"gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f

    def open(self):
        return self.gripper(GRIP["open_m"])

    def close(self):
        return self.gripper(GRIP["closed_m"])
