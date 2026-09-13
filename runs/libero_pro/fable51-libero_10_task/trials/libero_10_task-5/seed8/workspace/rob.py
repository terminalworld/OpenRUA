#!/usr/bin/env python3
"""Robot helper library for this Panda workstation.

World frame = 'world'; MoveIt IK/FK operate in the arm base frame
(panda_link0), which sits at BASE_IN_WORLD. All public functions take
and return WORLD-frame poses.

Import and use:
    from rob import Robot
    r = Robot()
    r.joints()                 -> dict name->pos
    r.hand_pose()              -> (xyz, quat_xyzw) of panda_hand in world (FK)
    r.ik(xyz, quat)            -> list of 7 joint positions or None
    r.move_joints(q, seconds)  -> error_code
    r.move_pose(xyz, quat, seconds) -> (ok, info)
    r.gripper(width_per_finger)-> (reached_goal, stalled, gap)
    r.wrench()                 -> (force xyz, torque xyz)
"""
import math
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # FK/IK model frame == world here (verified)
ARM = [f"panda_joint{i}" for i in range(1, 8)]
FINGERS = ["panda_finger_joint1", "panda_finger_joint2"]
TCP_OFFSET = 0.1034  # hand frame -> fingertip point along hand +z


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    m = R
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# Hand orientation with fingers along world X and approach (hand +z)
# pointing straight down: R = rot_x(pi) gives hand x=+X, y=-Y, z=-Z.
R_DOWN = rot_x(math.pi)


def hand_R(yaw=0.0, tilt_axis=None, tilt=0.0):
    """Top-down hand, yawed about world Z by `yaw` (finger axis = X
    rotated by yaw), then tilted about a world axis by `tilt` rad."""
    R = rot_z(yaw) @ R_DOWN
    if tilt_axis is not None and tilt != 0.0:
        ax = np.asarray(tilt_axis, float)
        ax = ax / np.linalg.norm(ax)
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        Rt = np.eye(3) + math.sin(tilt) * K + (1 - math.cos(tilt)) * K @ K
        R = Rt @ R
    return R


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")

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
        end = time.time() + 10
        while self._js is None and time.time() < end:
            self.spin(0.1)
        if self._js is None:
            raise RuntimeError("no /joint_states")
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j[FINGERS[0]] - j[FINGERS[1]] if j[FINGERS[1]] < 0 else j[FINGERS[0]] + j[FINGERS[1]]

    def wrench(self):
        self._wr = None
        end = time.time() + 5
        while self._wr is None and time.time() < end:
            self.spin(0.1)
        if self._wr is None:
            return None
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def fk(self, q=None, link="panda_hand"):
        if q is None:
            q = self.arm_q()
        if not self.fk_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no FK service")
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return xyz, q

    def hand_pose(self):
        return self.fk()

    def tcp_pose(self):
        xyz, q = self.fk()
        R = quat_to_R(q)
        return xyz + TCP_OFFSET * R[:, 2], q

    # ---------------- IK ----------------
    def ik(self, xyz_world, quat_xyzw, seed=None, attempts=3, timeout=2.0):
        if not self.ik_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no IK service")
        if seed is None:
            seed = self.arm_q()
        p_base = np.asarray(xyz_world, float) - BASE_IN_WORLD
        # IK tip link is panda_link8; panda_hand = link8 * rot_z(-pi/4)
        # (same origin). Convert the requested HAND orientation.
        R8 = quat_to_R(quat_xyzw) @ rot_z(math.pi / 4)
        quat_xyzw = R_to_quat(R8)
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "panda_arm"
            req.ik_request.pose_stamped.header.frame_id = ""
            pp = req.ik_request.pose_stamped.pose
            pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
            pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, quat_xyzw)
            req.ik_request.robot_state.joint_state.name = ARM
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[n] for n in ARM]
        return None

    def ik_tcp(self, tcp_xyz_world, R, **kw):
        """IK for a fingertip (TCP) position with hand rotation matrix R."""
        R = np.asarray(R)
        hand_xyz = np.asarray(tcp_xyz_world, float) - TCP_OFFSET * R[:, 2]
        return self.ik(hand_xyz, R_to_quat(R), **kw)

    # ---------------- motion ----------------
    def move_joints(self, q, seconds=3.0, waypoints=None):
        """Send one trajectory (optionally several waypoints, each a
        (q, t) pair) and wait. Returns error_code (0 = ok)."""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if waypoints is None:
            waypoints = [(q, seconds)]
        for qq, t in waypoints:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("FJT goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = r.result.error_code if r is not None else -99
        return code

    def move_pose(self, xyz_world, quat_xyzw, seconds=3.0, seed=None):
        q = self.ik(xyz_world, quat_xyzw, seed=seed)
        if q is None:
            return False, "IK failed"
        code = self.move_joints(q, seconds)
        now = np.array(self.arm_q())
        err = np.abs(now - np.array(q)).max()
        if err > 0.01:  # controller lag: re-send the same goal once to converge
            code = self.move_joints(q, max(1.5, seconds / 2))
            now = np.array(self.arm_q())
            err = np.abs(now - np.array(q)).max()
        xyz_now, _ = self.fk()
        cart = np.linalg.norm(xyz_now - np.asarray(xyz_world, float))
        return code == 0 and err < 0.02, f"code={code} max_joint_err={err:.4f} cart_err={cart*1000:.1f}mm"

    def move_tcp(self, tcp_xyz_world, R, seconds=3.0, seed=None):
        R = np.asarray(R)
        hand_xyz = np.asarray(tcp_xyz_world, float) - TCP_OFFSET * R[:, 2]
        return self.move_pose(hand_xyz, R_to_quat(R), seconds, seed)

    def gripper(self, width, effort=30.0):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(effort)
        send = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        rf = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        res = rf.result().result
        j = self.joints()
        return res.reached_goal, res.stalled, (j[FINGERS[0]], j[FINGERS[1]])

    def close(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _minjerk(s):
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def move_tcp_smooth(r, p0, p1, R, seconds, n=12):
    """Straight-line TCP move with a minimum-jerk time profile sent as one
    multi-waypoint trajectory (bounded acceleration -> no swing of a hinged
    object in the grasp). Returns (ok, info)."""
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    q = r.arm_q(); wps = []
    for i in range(1, n + 1):
        s = i / n
        p = p0 + _minjerk(s) * (p1 - p0)
        q = r.ik_tcp(p, R, seed=q)
        if q is None:
            return False, f"IK failed at waypoint {i}"
        wps.append((q, seconds * s))
    code = r.move_joints(wps[-1][0], seconds, waypoints=wps)
    now = np.array(r.arm_q()); err = np.abs(now - np.array(wps[-1][0])).max()
    if err > 0.01:
        code = r.move_joints(wps[-1][0], 1.5)
        now = np.array(r.arm_q()); err = np.abs(now - np.array(wps[-1][0])).max()
    cart = np.linalg.norm(r.tcp_pose()[0] - p1)
    return code == 0 and err < 0.02, f"code={code} max_joint_err={err:.4f} cart_err={cart*1000:.1f}mm"
