#!/usr/bin/env python3
"""Persistent helper: FK/IK, trajectories, gripper, joint/wrench reads. World frame."""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
# Verified: /compute_fk and /compute_ik poses are already WORLD-frame here
# (DH FK + camera cross-check), despite the docs' base-frame note.
BASE = np.array([0.0, 0.0, 0.0])
TCP = M["hand"]["tcp_offset_m"]
HAND = "panda_hand"


def q_down(yaw_deg=0.0):
    """Quaternion (x,y,z,w) for hand pointing straight down; yaw about world z.
    yaw=0 -> fingers open along world y; yaw=90 -> along world x."""
    R = Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])
    return R.as_quat()


class Robot:
    def __init__(self, name="robot_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, m):
        self.js = m

    def _on_wr(self, m):
        self.wr = m

    def spin(self, t=0.3):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self.spin(0.3)
        d = dict(zip(self.js.name, self.js.position))
        return np.array([d[j] for j in ARM])

    def fingers(self):
        self.spin(0.3)
        d = dict(zip(self.js.name, self.js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self.spin(0.3)
        if self.wr is None:
            return None
        f = self.wr.wrench.force
        t = self.wrench_t = self.wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    # ---------- kinematics ----------
    def _seed(self, q=None):
        q = self.joints() if q is None else q
        s = JointState()
        s.name = list(ARM)
        s.position = [float(v) for v in q]
        return s

    def fk(self, q=None, tcp=True):
        """World pose (xyz, quat xyzw) of hand (or TCP)."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [HAND]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        if tcp:
            xyz = xyz + Rot.from_quat(q).as_matrix()[:, 2] * TCP
        return xyz, q

    def ik(self, xyz, quat, tcp=True, seed=None, attempts=3):
        """Joint solution for world pose of TCP (or hand). Returns np.array or None."""
        xyz = np.asarray(xyz, float)
        if tcp:
            xyz = xyz - Rot.from_quat(quat).as_matrix()[:, 2] * TCP
        p_base = xyz - BASE
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.ik_link_name = HAND
            req.ik_request.pose_stamped.header.frame_id = ""
            pp = req.ik_request.pose_stamped.pose
            pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
            pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state = self._seed(seed)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return np.array([sol[j] for j in ARM])
        return None

    # ---------- motion ----------
    def move_joints(self, waypoints, seconds):
        """waypoints: list of joint arrays; seconds: total time (spread evenly) or list."""
        waypoints = [np.asarray(w, float) for w in waypoints]
        if np.isscalar(seconds):
            times = np.linspace(0, seconds, len(waypoints) + 1)[1:]
        else:
            times = list(seconds)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for w, t in zip(waypoints, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("FJT goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = np.abs(self.joints() - waypoints[-1]).max()
        return code, err

    def move_tcp(self, xyz, quat, seconds=3.0, n=1, seed=None):
        """Straight-ish line to TCP pose via n IK waypoints. Returns (code, joint_err, final_tcp_err)."""
        start, _ = self.fk()
        wps = []
        seed_q = self.joints() if seed is None else seed
        for i in range(1, n + 1):
            p = start + (np.asarray(xyz) - start) * i / n
            q = self.ik(p, quat, seed=seed_q)
            if q is None:
                raise RuntimeError(f"IK failed at waypoint {i}/{n}: {p}")
            wps.append(q)
            seed_q = q
        code, jerr = self.move_joints(wps, seconds)
        now, _ = self.fk()
        return code, jerr, np.asarray(xyz) - now

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def open(self):
        return self.gripper(GRIP["open_m"])

    def close(self):
        return self.gripper(GRIP["closed_m"])
