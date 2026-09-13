"""Shared helpers: joint state, FK/IK (world frame, panda_hand), trajectories, gripper, snapshots."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

import cv2

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE = np.array([0.0, 0.0, 0.0])  # FK/IK poses come back in world already (SRDF virtual joint world->panda_link0 is populated)
FINGERS = ["panda_finger_joint1", "panda_finger_joint2"]


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
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def R_from_axes(x_hand, y_hand, z_hand):
    """Rotation whose columns are the hand axes expressed in world."""
    R = np.column_stack([x_hand, y_hand, z_hand])
    assert np.linalg.det(R) > 0.99, np.linalg.det(R)
    return R


R_DOWN = quat_to_R([1, 0, 0, 0])  # hand z pointing down (-Z world), fingers along world y


class Robot:
    def __init__(self, name="robot_lib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.bridge = CvBridge()
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def _on_js(self, msg):
        self._js = msg

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger_gap(self):
        d = self.joints()
        return d[FINGERS[0]] - d[FINGERS[1]]  # joint2 reads negative here

    def fk(self, q=None, link="panda_hand"):
        """World pose (p, R) of link for arm config q (default: current)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        ps = res.pose_stamped[0].pose
        p = np.array([ps.position.x, ps.position.y, ps.position.z]) + BASE
        R = quat_to_R([ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w])
        return p, R

    def ik(self, p_world, R, seed=None, link="panda_hand", timeout=2.0):
        """Arm joints placing the hand frame at (p_world, R). None if no solution."""
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = link
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.pose_stamped.header.frame_id = ""
        pose = req.ik_request.pose_stamped.pose
        pb = np.asarray(p_world, float) - BASE
        pose.position.x, pose.position.y, pose.position.z = map(float, pb)
        q = R_to_quat(R)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK service timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    # ---------- acting ----------
    def move(self, waypoints, times):
        """Send one trajectory through joint waypoints at cumulative times (s)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(waypoints, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
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
            raise RuntimeError("trajectory result timeout")
        code = rf.result().result.error_code
        q_now = np.array(self.arm_q())
        err = np.abs(q_now - np.array(waypoints[-1])).max()
        print(f"  move: error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        rf = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap

    def snap(self, cam, out=None):
        topic = f"/{cam}/color/image_raw"
        got = []
        sub = self.node.create_subscription(Image, topic, got.append, 1)
        while not got:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, self.bridge.imgmsg_to_cv2(got[0], "bgr8"))
        return out

    # ---------- helpers ----------
    def cartesian_waypoints(self, p0, p1, R, n, seed):
        """IK along a straight line p0->p1 (n segments) at fixed R; returns joint list."""
        qs = []
        q = seed
        for i in range(1, n + 1):
            p = p0 + (p1 - p0) * i / n
            q = self.ik(p, R, seed=q)
            if q is None:
                raise RuntimeError(f"IK failed at waypoint {i}/{n}: {p}")
            qs.append(q)
        return qs

    def report(self, tag=""):
        p, R = self.fk()
        print(f"  [{tag}] hand p={np.round(p,4)} z_axis={np.round(R[:,2],3)} y_axis={np.round(R[:,1],3)} gap={self.finger_gap():.4f}", flush=True)
        return p, R
