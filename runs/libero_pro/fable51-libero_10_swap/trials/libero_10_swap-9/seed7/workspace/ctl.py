#!/usr/bin/env python3
"""Reusable control helpers for this Panda: IK, trajectories, gripper, TF.

Import from scripts; all poses are WORLD frame unless noted.
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
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE_T = np.array([0.0, 0.0, 0.0])  # measured: IK/FK services here work in WORLD coords
# IK tip link is panda_link8, which is panda_hand rotated +45deg about z
HAND_TO_LINK8 = Rot.from_euler("z", 45, degrees=True)


def log(*a):
    print(*a, flush=True)


class Robot:
    def __init__(self, name="ctl"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.wait_js()

    def _on_js(self, m):
        self.js = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self, timeout=20):
        end = time.time() + timeout
        self.js = None
        while self.js is None and time.time() < end:
            self.spin(0.2)
        if self.js is None:
            raise RuntimeError("no /joint_states")
        return self.js

    def joints(self, fresh=True):
        js = self.wait_js() if fresh else self.js
        d = dict(zip(js.name, js.position))
        return [d[j] for j in ARM]

    def fingers(self, fresh=True):
        js = self.wait_js() if fresh else self.js
        d = dict(zip(js.name, js.position))
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    # ---------- kinematics ----------
    def hand_pose(self):
        """world -> panda_hand as (xyz, quat xyzw) via TF."""
        end = time.time() + 10
        while time.time() < end:
            self.spin(0.1)
            if self.tfbuf.can_transform("world", "panda_hand", rclpy.time.Time()):
                break
        t = self.tfbuf.lookup_transform("world", "panda_hand", rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), np.array([q.x, q.y, q.z, q.w])

    def fk(self, joints):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in joints]
        self.fk_cli.wait_for_service(timeout_sec=10)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_T
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return xyz, q

    def ik(self, xyz_world, quat, seed=None, at_tcp=False, timeout=30, attempts=3):
        """IK for the panda_hand frame (or fingertip point when at_tcp)."""
        xyz = np.array(xyz_world, dtype=float)
        if at_tcp:
            R = Rot.from_quat(quat).as_matrix()
            xyz = xyz - TCP * R[:, 2]
        xyz_base = xyz - BASE_T
        seed = self.joints() if seed is None else list(seed)
        self.ik_cli.wait_for_service(timeout_sec=10)
        for k in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, xyz_base)
            q8 = (Rot.from_quat(quat) * HAND_TO_LINK8).as_quat()
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1, nanosec=0)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
            code = None if res is None else res.error_code.val
            log(f"  ik attempt {k+1} failed code={code} for {np.round(xyz,3)}")
            # perturb seed slightly for another try
            seed = [s + np.random.uniform(-0.2, 0.2) for s in seed]
        return None

    # ---------- motion ----------
    def move_joints(self, waypoints, times, hold=4.0):
        """waypoints: list of 7-lists; times: cumulative seconds per point.
        The controller lags; a `hold` segment at the final target lets it settle
        while the sim clock runs (without it goals end with tolerance errors)."""
        if any(wp is None for wp in waypoints):
            raise RuntimeError("move_joints got a None waypoint (IK failed)")
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        waypoints = list(waypoints) + [waypoints[-1]]
        times = list(times) + [times[-1] + hold]
        for wp, t in zip(waypoints, times):
            for j, (v, lim) in enumerate(zip(wp, LIMITS)):
                if not (lim[0] - 1e-6 <= v <= lim[1] + 1e-6):
                    raise RuntimeError(f"joint {j+1} target {v:.3f} outside {lim}")
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)
        if rf.result() is None:
            log("  trajectory result timeout (client side)")
            return None
        code = rf.result().result.error_code
        err = np.array(self.joints()) - np.array(waypoints[-1])
        log(f"  traj done code={code} max_joint_err={np.abs(err).max():.4f}")
        return code

    def move_pose(self, xyz, quat, seconds=None, at_tcp=False, seed=None, hold=4.0):
        q = self.ik(xyz, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            raise RuntimeError(f"IK failed for {np.round(xyz,3)}")
        if seconds is None:
            dist = np.abs(np.array(q) - np.array(self.joints())).max()
            seconds = max(2.0, dist / 0.5)
        code = self.move_joints([q], [seconds], hold=hold)
        return q, code

    def move_path(self, poses, quat, seconds_per=2.0, at_tcp=False, first_seconds=None):
        """Sequence of cartesian waypoints -> one joint trajectory."""
        seed = self.joints()
        wps, times = [], []
        t = 0.0
        for i, xyz in enumerate(poses):
            q = self.ik(xyz, quat, seed=seed, at_tcp=at_tcp)
            if q is None:
                raise RuntimeError(f"IK failed for waypoint {i} {np.round(xyz,3)}")
            # continuity check
            jump = np.abs(np.array(q) - np.array(seed)).max()
            if jump > 1.5:
                log(f"  warning: big joint jump {jump:.2f} at waypoint {i}")
            wps.append(q)
            t += (first_seconds if (i == 0 and first_seconds) else seconds_per)
            times.append(t)
            seed = q
        return self.move_joints(wps, times)

    def gripper(self, width):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        res = rf.result().result
        f = self.fingers()
        log(f"  gripper -> reached={res.reached_goal} stalled={res.stalled} fingers={f}")
        return f

    def servo(self, lin, ang=(0, 0, 0), ticks=20, frame=None):
        msg = TwistStamped()
        msg.header.frame_id = frame or TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            self.twist_pub.publish(msg)
            self.spin(0.05)


def quat_down(yaw_deg=0.0):
    """Hand pointing straight down; yaw rotates the finger axis about world z.
    yaw=0: fingers close along world y (hand x = world x)."""
    return (Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])).as_quat()


def quat_from_axes(approach, finger_axis):
    """Quaternion for a hand whose z (approach) and y (finger closing) axes are given in world."""
    z = np.array(approach, float); z /= np.linalg.norm(z)
    y = np.array(finger_axis, float); y -= z * (y @ z); y /= np.linalg.norm(y)
    x = np.cross(y, z)
    R = np.stack([x, y, z], axis=1)
    return Rot.from_matrix(R).as_quat()
