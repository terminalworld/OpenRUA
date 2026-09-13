#!/usr/bin/env python3
"""Reusable robot helpers: joint state, FK, IK, trajectory, gripper, TF, camera.
World frame = panda_link0 + (-0.66, 0, 0.912).
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Image, CameraInfo
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
LIMITS = np.array(FJT["limits_rad"])
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])
TCP = M["hand"]["tcp_offset_m"]


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)

    def _on_js(self, msg):
        self.js = msg

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self.js = None
            end = time.time() + 20
            while self.js is None and time.time() < end:
                self.spin(0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d

    def q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def fk_pose(self, q=None, link="panda_hand"):
        """Return (pos_world, quat_xyzw) of link for joints q (default current)."""
        if q is None:
            q = self.q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])  # FK is already in world
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def tcp_pose(self, q=None):
        pos, quat = self.fk_pose(q)
        R = Rot.from_quat(quat).as_matrix()
        return pos + TCP * R[:, 2], quat

    # ---------- IK ----------
    def solve_ik(self, pos_world, quat_xyzw, seed=None, at_tcp=True, tries=1):
        """IK for hand (or TCP) pose in world; returns joint array or None."""
        pos = np.array(pos_world, float)
        R = Rot.from_quat(quat_xyzw).as_matrix()
        if at_tcp:
            pos = pos - TCP * R[:, 2]
        pos_base = pos  # IK model frame == world (verified by FK roundtrip)
        if seed is None:
            seed = self.q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos_base)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat_xyzw)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(x) for x in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                qs = np.array([sol[j] for j in ARM])
                # verify: some IK plugins return approximate solutions
                fp, fq = self.fk_pose(qs)
                dpos = np.linalg.norm(fp - pos_base)
                dang = (Rot.from_quat(fq) * Rot.from_quat(quat_xyzw).inv()).magnitude()
                if dpos < 0.003 and dang < np.radians(2):
                    return qs
                self.node.get_logger().warn(f"IK approx rejected dpos={dpos:.4f} dang={np.degrees(dang):.1f}")
            seed = np.clip(seed + np.random.uniform(-0.3, 0.3, 7), LIMITS[:, 0], LIMITS[:, 1])
        return None

    # ---------- acting ----------
    def move_joints(self, q_target, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if waypoints:
            n = len(waypoints) + 1
            for i, wp in enumerate(waypoints):
                t = seconds * (i + 1) / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in wp])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q_target])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        handle = send.result()
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        qn = self.q()
        err = np.abs(qn - np.array(q_target)).max()
        return code, err

    def move_tcp(self, pos_world, quat_xyzw, seconds=3.0, seed=None, tries=5):
        q = self.solve_ik(pos_world, quat_xyzw, seed=seed, tries=tries)
        if q is None:
            return None
        code, err = self.move_joints(q, seconds)
        return code, err, q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        return r.reached_goal, r.stalled, self.fingers()

    # ---------- camera ----------
    def snap(self, cam, out=None):
        from cv_bridge import CvBridge
        import cv2
        got = {}
        topic = f"/{cam}/color/image_raw"
        sub = self.node.create_subscription(Image, topic, lambda m: got.setdefault("m", m), 1)
        end = time.time() + 30
        while "m" not in got and time.time() < end:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        img = CvBridge().imgmsg_to_cv2(got["m"], "bgr8")
        out = out or f"{cam}.png"
        cv2.imwrite(out, img)
        return out


def quat_down(yaw_deg=0.0):
    """Quaternion (xyzw) for hand pointing straight down (z axis = -world z),
    with the finger-closing axis (hand y) rotated by yaw about world z.
    At yaw=0, hand x = world x, hand y = -world y."""
    return Rot.from_euler("xyz", [180, 0, yaw_deg], degrees=True).as_quat()


def quat_from_axes(z_axis, x_axis):
    z = np.array(z_axis, float); z /= np.linalg.norm(z)
    x = np.array(x_axis, float); x -= x.dot(z) * z; x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return Rot.from_matrix(np.c_[x, y, z]).as_quat()
