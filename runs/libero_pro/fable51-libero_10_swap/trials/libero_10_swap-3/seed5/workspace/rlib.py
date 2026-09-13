#!/usr/bin/env python3
"""Small helper layer over MoveIt/ros2_control for this Panda.

World frame = panda_link0 + BASE_OFF. All public poses are WORLD frame.
"""
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, WrenchStamped
from moveit_msgs.msg import JointConstraint
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_OFF = np.zeros(3)  # MoveIt model frame here IS world (verified: FK(panda_link0) = (-0.66,0,0.912))
TCP = M["hand"]["tcp_offset_m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    """Rotation matrix -> quaternion (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


def hand_R(finger_axis_world, approach_world):
    """Hand rotation: hand Z = approach dir, hand Y = finger closing axis."""
    z = np.asarray(approach_world, float); z /= np.linalg.norm(z)
    y = np.asarray(finger_axis_world, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.stack([x, y, z], axis=1)


def top_down_R(yaw=0.0):
    """Hand pointing straight down, finger axis rotated `yaw` from world x."""
    return hand_R([np.cos(yaw), np.sin(yaw), 0.0], [0, 0, -1])


class Robot:
    def __init__(self, name="rlib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = {}
        self.wr = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.cp_cli = self.node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        for c, n in [(self.fk_cli, "fk"), (self.ik_cli, "ik"), (self.cp_cli, "cartesian")]:
            if not c.wait_for_service(timeout_sec=20):
                raise SystemExit(f"{n} service unavailable")
        if not self.fjt.wait_for_server(timeout_sec=20):
            raise SystemExit("no FJT server")
        if not self.grip.wait_for_server(timeout_sec=20):
            raise SystemExit("no gripper server")

    def _on_js(self, m):
        self.js = dict(zip(m.name, m.position))

    def _on_wr(self, m):
        self.wr = np.array([m.wrench.force.x, m.wrench.force.y, m.wrench.force.z,
                            m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z])

    def spin(self, t=0.3):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self, fresh=True):
        if fresh:
            self.js = {}
            while not self.js:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        return self.js

    def arm_q(self):
        js = self.joints()
        return np.array([js[j] for j in ARM])

    def finger(self):
        js = self.joints()
        return js.get("panda_finger_joint1", float("nan"))

    def wrench(self):
        self.wr = {}
        end = time.time() + 5
        while isinstance(self.wr, dict) and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return self.wr

    def _seed_state(self, q=None):
        js = JointState()
        q = self.arm_q() if q is None else np.asarray(q)
        js.name = list(ARM)
        js.position = [float(v) for v in q]
        return js

    def _call(self, cli, req, timeout=60):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    # ---------- kinematics (world frame) ----------
    def fk(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed_state(q)
        res = self._call(self.fk_cli, req)
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_OFF
        R = quat_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, R

    def tcp(self, q=None):
        pos, R = self.fk(q)
        return pos + TCP * R[:, 2], R

    def _pose_msg(self, pos_world, R):
        p = Pose()
        pb = np.asarray(pos_world) - BASE_OFF
        p.position.x, p.position.y, p.position.z = map(float, pb)
        qx, qy, qz, qw = R_quat(R)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, (qx, qy, qz, qw))
        return p

    J2MAX = 0.45  # the sim's joint2 stalls at ~0.47 rad whatever the URDF says

    def ik(self, pos_world, R, seed=None, at_tcp=True, timeout=30, avoid_collisions=False, j2max=None):
        """j2max: cap panda_joint2 via a joint constraint (default J2MAX; None/inf disables)."""
        pos = np.asarray(pos_world, float)
        if at_tcp:
            pos = pos - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.pose_stamped.pose = self._pose_msg(pos, R)
        req.ik_request.robot_state.joint_state = self._seed_state(seed)
        req.ik_request.avoid_collisions = avoid_collisions
        j2max = self.J2MAX if j2max is None else j2max
        if np.isfinite(j2max):
            jc = JointConstraint(joint_name="panda_joint2", position=float((j2max - 1.76) / 2),
                                 tolerance_above=float(j2max - (j2max - 1.76) / 2),
                                 tolerance_below=float((j2max - 1.76) / 2 + 1.76), weight=1.0)
            req.ik_request.constraints.joint_constraints.append(jc)
        req.ik_request.timeout = Duration(sec=int(timeout % 60 if timeout < 60 else 5))
        res = self._call(self.ik_cli, req, timeout=timeout + 10)
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            raise RuntimeError(f"IK failed code={res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    def cartesian(self, waypoints, start_q=None, at_tcp=True, step=0.005, avoid_collisions=False):
        """waypoints: list of (pos_world, R). Returns (joint positions list, fraction)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = ""
        req.start_state.joint_state = self._seed_state(start_q)
        req.group_name = M["planning"]["group"]
        req.link_name = "panda_hand"
        for pos, R in waypoints:
            pos = np.asarray(pos, float)
            if at_tcp:
                pos = pos - TCP * R[:, 2]
            req.waypoints.append(self._pose_msg(pos, R))
        req.max_step = step
        req.jump_threshold = 0.0
        req.avoid_collisions = avoid_collisions
        req.max_velocity_scaling_factor = 0.5
        req.max_acceleration_scaling_factor = 0.5
        res = self._call(self.cp_cli, req, timeout=120)
        if res is None:
            raise RuntimeError("cartesian path: no answer")
        if res.error_code.val != 1:
            raise RuntimeError(f"cartesian path failed code={res.error_code.val}")
        jt = res.solution.joint_trajectory
        idx = [jt.joint_names.index(j) for j in ARM]
        pts = [np.array([p.positions[i] for i in idx]) for p in jt.points]
        return pts, res.fraction

    # ---------- motion ----------
    VMAX = 0.12  # rad/s: the sim controller tracks ~0.2 rad/s at most; stay under

    def move_q(self, targets, seconds=None, timeout=900, retry=True):
        """Send one trajectory through the given joint positions (list of arrays).
        seconds=None -> duration from path length at VMAX."""
        if isinstance(targets, np.ndarray) and targets.ndim == 1:
            targets = [targets]
        q0 = self.arm_q()
        path = [q0] + [np.asarray(t) for t in targets]
        length = max(sum(abs(path[i + 1][j] - path[i][j]) for i in range(len(path) - 1)) for j in range(len(ARM)))
        auto = max(1.0, length / self.VMAX)
        if seconds is None or seconds < auto:
            seconds = auto
        print(f"  move: max joint path {length:.3f} rad over {seconds:.1f}s")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        n = len(targets)
        for i, q in enumerate(targets):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("FJT goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        if rf.result() is None:
            raise RuntimeError("FJT result timeout")
        code = rf.result().result.error_code
        q = self.arm_q()
        err = np.abs(q - np.asarray(targets[-1])).max()
        print(f"  move done code={code} max_joint_err={err:.4f}")
        if err > 0.01 and retry:
            print("  -> re-sending final target to converge")
            return self.move_q(targets[-1], None, timeout, retry=False)
        return code, err

    def move_cart(self, waypoints, seconds=None, at_tcp=True, min_fraction=0.99):
        pts, frac = self.cartesian(waypoints, at_tcp=at_tcp)
        print(f"  cartesian fraction={frac:.3f} points={len(pts)}")
        if frac < min_fraction:
            raise RuntimeError(f"cartesian fraction {frac:.3f} < {min_fraction}")
        return self.move_q(pts, seconds)

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        rf = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        print(f"  gripper -> pos={r.position:.4f} reached={r.reached_goal} stalled={r.stalled} finger={self.finger():.4f}")
        return r

    def report(self):
        q = self.arm_q()
        pos, R = self.fk(q)
        t = pos + TCP * R[:, 2]
        print(f"  q={np.round(q,3).tolist()}")
        print(f"  hand={np.round(pos,4).tolist()} tcp={np.round(t,4).tolist()} finger={self.finger():.4f}")
        print(f"  hand Z={np.round(R[:,2],3).tolist()} Y(finger axis)={np.round(R[:,1],3).tolist()}")
        return q, pos, R
