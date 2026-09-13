"""Small helper layer over this machine's ROS graph (see machine.yaml).

World <-> base: panda_link0 sits at world (-0.66, 0, 0.912), no rotation.
IK/FK talk to MoveIt in the base frame with an EMPTY frame_id.
"""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
LIMITS = FJT["limits_rad"]
# Empirically (FK probe vs. camera TF) the planner's model frame IS the
# world frame on this machine: FK of panda_hand at the start pose returns
# (-0.203, 0, 1.27) which matches the eye-in-hand camera's world TF.
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])
TCP = float(M["hand"]["tcp_offset_m"])


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    # returns (x, y, z, w)
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def R_from_axes(z_axis, y_axis):
    """Hand rotation with given approach (hand z) and finger-closing (hand y)
    directions, both in world; x completes the right-handed frame."""
    z = np.asarray(z_axis, float); z /= np.linalg.norm(z)
    y = np.asarray(y_axis, float); y -= z * (y @ z); y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


class Robot:
    def __init__(self, name="rlib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.update(zip(m.name, m.position)), 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)

    # ---------- sensing ----------
    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def js(self, fresh=True):
        if fresh:
            self._js.clear()
        while not all(j in self._js for j in ARM):
            self.spin(0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.js()
        return [js[j] for j in ARM]

    def fingers(self):
        js = self.js()
        return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    def fk(self, q=None, link="panda_hand"):
        """Pose of link in WORLD: (pos[3], R[3x3])."""
        if q is None:
            q = self.arm_q()
        self.fk_cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        R = quat_to_R((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
        return pos, R

    # ---------- planning ----------
    def ik(self, pos_world, R, seed=None, tries=8, timeout=2.0, max_jump=None):
        """Joint solution for HAND frame at pos_world/R. Returns list or None.
        max_jump: reject solutions farther than this (rad, per joint) from the
        seed; the seed is perturbed slightly between retries."""
        if seed is None:
            seed = self.arm_q()
        seed0 = np.array(seed, float)
        self.ik_cli.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = np.asarray(pos_world, float) - BASE_IN_WORLD
        q = R_to_quat(R)
        pose = req.ik_request.pose_stamped.pose
        pose.position.x, pose.position.y, pose.position.z = map(float, p)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        rng = np.random.default_rng(0)
        best = None
        for k in range(tries):
            s = seed0 if k == 0 else seed0 + rng.normal(0, 0.05 * k, len(seed0))
            req.ik_request.robot_state.joint_state.position = [float(v) for v in s]
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                q = [sol[j] for j in ARM]
                jump = float(np.abs(np.array(q) - seed0).max())
                if max_jump is None or jump <= max_jump:
                    return q
                if best is None or jump < best[0]:
                    best = (jump, q)
        if best is not None:
            print(f"ik: only far solutions (min jump {best[0]:.2f} rad)")
        return None

    def ik_tcp(self, tcp_world, R, **kw):
        """IK with the target given for the fingertip point (TCP)."""
        hand = np.asarray(tcp_world, float) - TCP * R[:, 2]
        return self.ik(hand, R, **kw)

    # ---------- acting ----------
    def move(self, waypoints, seconds, verify=True):
        """waypoints: list of joint lists (or one), spaced evenly in time."""
        if not isinstance(waypoints[0], (list, tuple, np.ndarray)):
            waypoints = [waypoints]
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        n = len(waypoints)
        for i, wp in enumerate(waypoints):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = None
        if verify:
            q = np.array(self.arm_q())
            err = float(np.abs(q - np.array(waypoints[-1], float)).max())
        return code, err

    def gripper(self, width, wait=120):
        self.grip.wait_for_server(timeout_sec=10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=wait)
        r = res.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def servo(self, lin, ang=(0, 0, 0), ticks=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(dt)

    def line_ik(self, p0, p1, R, n, seed=None, max_jump=0.6):
        """IK along a straight world-space line for the HAND frame; returns
        list of joint solutions or raises on the first failure."""
        seed = list(seed if seed is not None else self.arm_q())
        out = []
        for i in range(1, n + 1):
            p = np.asarray(p0) + (np.asarray(p1) - np.asarray(p0)) * i / n
            q = self.ik(p, R, seed=seed, max_jump=max_jump)
            if q is None:
                raise RuntimeError(f"IK failed at waypoint {i}/{n}: {p}")
            out.append(q); seed = q
        return out


def path_check(r, q0, q1, n=12, links=("panda_hand", "panda_link4", "panda_link6", "panda_link7")):
    """FK of several links along the straight joint-space path q0->q1.
    Returns list of (s, {link: pos}) and prints the lowest points."""
    out = []
    q0 = np.asarray(q0, float); q1 = np.asarray(q1, float)
    for i in range(n + 1):
        s = i / n
        q = q0 + (q1 - q0) * s
        d = {}
        for L in links:
            p, R = r.fk(q, link=L)
            if L == "panda_hand":
                d["tcp"] = p + TCP * R[:, 2]
            d[L] = p
        out.append((s, d))
    return out


def print_path(out):
    for s, d in out:
        print(f"s={s:.2f} " + " ".join(f"{k}=({v[0]:+.2f},{v[1]:+.2f},{v[2]:.2f})" for k, v in d.items()))


def go_tcp(r, tcp, R, n=3, rate=0.6, min_t=2.0, mj=0.7, seed=None):
    """Straight-line TCP move (hand orientation R) with duration scaled to the
    joint distance; resends once if the controller stops short."""
    p0, _ = r.fk()
    hand = np.asarray(tcp, float) - TCP * R[:, 2]
    wps = r.line_ik(p0, hand, R, n, seed=seed, max_jump=mj)
    q0 = np.array(r.arm_q())
    dist = float(np.abs(np.array(wps[-1]) - q0).max())
    t = max(min_t, dist / rate)
    code, err = r.move(wps, t)
    if err > 0.02:
        code, err = r.move(wps[-1], max(1.5, err / rate))
    p1, R1 = r.fk()
    print(f"  tcp -> {np.round(p1 + TCP * R1[:, 2], 3)} code {code} err {err:.4f}")
    return wps[-1]


def go_joint(r, q1, rate=0.25, n=None, min_t=2.0):
    """Joint-space move with dense waypoints, slow enough for the controller."""
    q0 = np.array(r.arm_q()); q1 = np.array(q1, float)
    dist = float(np.abs(q1 - q0).max())
    t = max(min_t, dist / rate)
    n = n or max(2, int(np.ceil(t)))
    wps = [list(q0 + (q1 - q0) * (i + 1) / n) for i in range(n)]
    code, err = r.move(wps, t)
    if err > 0.02:
        code, err = r.move(list(q1), max(1.5, err / rate))
    print(f"  joint move code {code} err {err:.4f} ({t:.1f}s)")
    return code, err
