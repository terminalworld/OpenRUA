#!/usr/bin/env python3
"""Small helper layer over the machine's ROS ports (built from machine.yaml).

Frames: world -> panda_link0 is a pure translation BASE_T (read from TF once);
poses given here are in WORLD and converted to the base frame for MoveIt.
"""
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
# NOTE: verified empirically: /compute_fk and /compute_ik with empty frame_id
# work in the WORLD frame (FK of the start pose gives z=1.27, matching the
# birdview), so no base shift is applied.
BASE_T = np.zeros(3)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    """rotation matrix -> (x,y,z,w)"""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()


# hand orientations (world == base orientation). Hand z = approach axis,
# hand y = finger closing axis.
def R_topdown(close_axis="y"):
    """hand pointing straight down; fingers close along world x or y."""
    if close_axis == "y":
        return np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)
    return np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)


def R_from_axes(z_axis, y_axis):
    z = np.asarray(z_axis, float); z /= np.linalg.norm(z)
    y = np.asarray(y_axis, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.stack([x, y, z], axis=1)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self._wr = None
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")

    def _on_js(self, m): self._wr_dummy = None; self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.1):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------------- sensing
    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 5
        while self._wr is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self._wr is None:
            return None
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def fk_world(self, q=None, link="panda_hand"):
        """hand pose in WORLD: (pos, R)."""
        if q is None:
            q = self.arm_q()
        if not self.fk.wait_for_service(timeout_sec=10):
            raise RuntimeError("no FK service")
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_T
        R = quat_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, R

    def tcp_world(self, q=None):
        pos, R = self.fk_world(q)
        return pos + TCP * R[:, 2], R

    # ---------------- IK
    def ik_world(self, pos, R, seed=None, tcp=True, attempts=1, timeout=1.0):
        """joint solution for hand (or tcp) pose in WORLD, or None."""
        pos = np.asarray(pos, float)
        if tcp:
            pos = pos - TCP * R[:, 2]
        pb = pos - BASE_T
        # IK tip link is panda_link8; panda_hand = link8 * Rz(-pi/4)
        c, s_ = np.cos(np.pi / 4), np.sin(np.pi / 4)
        R8 = R @ np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
        q = R_quat(R8)
        if seed is None:
            seed = self.arm_q()
        if not self.ik.wait_for_service(timeout_sec=10):
            raise RuntimeError("no IK service")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(x) for x in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        best = None
        for _ in range(attempts):
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                cand = np.array([sol[j] for j in ARM])
                if best is None or np.abs(cand - seed).sum() < np.abs(best - seed).sum():
                    best = cand
        return best

    # ---------------- acting
    def move_q(self, q, seconds=3.0, via=None, retries=2, tol=0.01):
        """one trajectory; via = list of (q, t) intermediate points."""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        for qq, tt in (via or []) + [(q, seconds)]:
            pt = JointTrajectoryPoint(positions=[float(x) for x in qq])
            pt.time_from_start = Duration(sec=int(tt), nanosec=int((tt % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        qa = self.arm_q()
        err = np.abs(qa - np.asarray(q)).max()
        print(f"  move done code={code} max_joint_err={err:.4f}", flush=True)
        # controller lag on big moves: resend the remainder (see docs)
        if err > tol and retries > 0:
            return self.move_q(q, max(1.5, seconds * min(1.0, err / 0.5)), retries=retries - 1, tol=tol)
        return code, err

    def move_tcp(self, pos, R, seconds=3.0, seed=None, attempts=3):
        q = self.ik_world(pos, R, seed=seed, attempts=attempts)
        if q is None:
            print(f"  IK FAILED for tcp {np.round(pos,3)}", flush=True)
            return None
        self.move_q(q, seconds)
        p, _ = self.tcp_world()
        print(f"  tcp now {np.round(p,4)} (target {np.round(pos,4)})", flush=True)
        return q

    def gripper(self, width):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}", flush=True)
        return f

    def snap(self, cam, out=None):
        from cv_bridge import CvBridge
        import cv2
        got = {}
        sub = self.node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("m", m), 1)
        end = time.time() + 30
        while "m" not in got and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        out = out or f"{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got["m"], desired_encoding="bgr8"))
        return out

    def depth_cloud(self, cam):
        """world-frame point cloud (H,W,3) from a camera's current depth."""
        from cv_bridge import CvBridge
        got = {}
        sub = self.node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("m", m), 1)
        end = time.time() + 30
        while "m" not in got and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        d = CvBridge().imgmsg_to_cv2(got["m"], desired_encoding="passthrough").astype(float)
        T = np.load(f"/workspace/{cam}_T.npy"); K = np.load(f"/workspace/{cam}_K.npy")
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        H, W = d.shape
        vs, us = np.mgrid[0:H, 0:W]
        P = np.stack([(us - cx) * d / fx, (vs - cy) * d / fy, d, np.ones_like(d)], -1) @ T.T
        return P[..., :3]
