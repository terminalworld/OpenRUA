#!/usr/bin/env python3
"""Reusable helpers for this Panda: joint state, FK, IK, trajectory, gripper.
World frame = panda_link0 + BASE offset (from TF: world->panda_link0).
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
# Verified: /compute_fk and /compute_ik already work in the WORLD frame
# (model root is 'world'; Panda home pose FK = (0.307,0,0.59)+(-0.51,0,0.42)).
BASE = np.array([0.0, 0.0, 0.0])
TCP = float(M["hand"]["tcp_offset_m"])
RZ45 = np.array([[math.cos(math.pi / 4), -math.sin(math.pi / 4), 0],
                 [math.sin(math.pi / 4), math.cos(math.pi / 4), 0], [0, 0, 1]])


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
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


def topdown_quat(yaw):
    """Hand z pointing down (-world z), hand x rotated by yaw about world z.
    Fingers open along hand y."""
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    Rflip = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # x->x, y->-y, z->-z
    return R_to_quat(Rz @ Rflip)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def _on_js(self, m):
        self._js["m"] = m
        self._js["t"] = time.time()

    def joints(self, fresh=True):
        """Return dict name->pos from a fresh joint state."""
        t0 = time.time()
        self._js.pop("m", None) if fresh else None
        while "m" not in self._js and time.time() - t0 < 10:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def _spin(self, fut, timeout):
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    def fk(self, q=None, link="panda_hand"):
        """Hand pose in WORLD frame: (pos[3], quat[4])."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        res = self._spin(self.fk_cli.call_async(req), 30)
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z,
                         p.orientation.w])
        return pos, quat

    def ik(self, pos_world, quat, seed=None, at_tcp=False, timeout=30):
        """IK for the hand (or TCP if at_tcp) at a world pose. Returns q or None."""
        pos = np.array(pos_world, dtype=float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(quat)[:, 2]
        pos = pos - BASE
        # IK tip is panda_link8 = panda_hand rotated +45deg about hand z
        quat = R_to_quat(quat_to_R(quat) @ RZ45)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = \
            map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = \
            [float(v) for v in (seed if seed is not None else self.arm_q())]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        res = self._spin(self.ik_cli.call_async(req), timeout)
        if res is None:
            print("IK: no answer", file=sys.stderr)
            return None
        if res.error_code.val != 1:
            print(f"IK failed code={res.error_code.val}", file=sys.stderr)
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_q(self, q, seconds=3.0, via=None, retries=2):
        """Send a trajectory (optionally through via points: list of (q, t))."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            for vq, vt in via:
                pt = JointTrajectoryPoint(positions=[float(v) for v in vq])
                pt.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        h = self._spin(self.fjt.send_goal_async(goal), 60)
        res = self._spin(h.get_result_async(), 600)
        code = res.result.error_code if res else None
        qn = self.arm_q()
        err = max(abs(a - b) for a, b in zip(qn, q))
        print(f"traj code={code} max_joint_err={err:.4f}")
        if err > 0.01 and retries > 0:  # controller lag: resend converges
            return self.move_q(q, max(1.5, seconds / 2), retries=retries - 1)
        return code, err

    def move_pose(self, pos_world, quat, seconds=3.0, at_tcp=True, seed=None):
        q = self.ik(pos_world, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            return None
        return self.move_q(q, seconds)

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        h = self._spin(self.grip.send_goal_async(g), 60)
        res = self._spin(h.get_result_async(), 300)
        r = res.result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"gap={self.finger_gap():.4f}")
        return r

    def tcp(self):
        pos, quat = self.fk()
        return pos + TCP * quat_to_R(quat)[:, 2], quat


# ---- eye-in-hand helpers -------------------------------------------------
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

EIH_OFF = np.array([0.050, 0.0, -0.001])      # camera in hand frame
EIH_R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])  # Rz(+90deg): hand->cam


def grab(node, topic, msg_type, timeout=15.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
    t0 = time.time()
    while "m" not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def eih_snap(r, tag="eih"):
    """Grab eye-in-hand color+depth; return (bgr, depth, K, T_world_cam)."""
    col = grab(r.node, "/robot0_eye_in_hand/color/image_raw", Image)
    dep = grab(r.node, "/robot0_eye_in_hand/depth/image_raw", Image)
    info = grab(r.node, "/robot0_eye_in_hand/color/camera_info", CameraInfo)
    br = CvBridge()
    bgr = br.imgmsg_to_cv2(col, "bgr8")
    depth = br.imgmsg_to_cv2(dep, "passthrough").astype(np.float32)
    K = np.array(info.k).reshape(3, 3)
    hpos, hq = r.fk()
    Rh = quat_to_R(hq)
    T = np.eye(4)
    T[:3, :3] = Rh @ EIH_R
    T[:3, 3] = hpos + Rh @ EIH_OFF
    cv2.imwrite(f"{tag}.png", bgr)
    np.save(f"{tag}_depth.npy", depth)
    return bgr, depth, K, T


def px_to_world(u, v, depth, K, T):
    z = float(depth[int(v), int(u)])
    p = np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, 1.0])
    return (T @ p)[:3]


def world_to_px(pw, K, T):
    pc = np.linalg.inv(T) @ np.array([*pw, 1.0])
    return (K[0, 0] * pc[0] / pc[2] + K[0, 2], K[1, 1] * pc[1] / pc[2] + K[1, 2])


def fit_circle(pts):
    """Least-squares circle fit to Nx2 points -> (cx, cy, r)."""
    x, y = pts[:, 0], pts[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    b = x * x + y * y
    c, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = c[0], c[1]
    r = math.sqrt(c[2] + cx * cx + cy * cy)
    return cx, cy, r


def rim_fit(depth, K, T, table_z, zmin, zmax, roi=None, tag=None, bgr=None):
    """World-fit of a mug rim: pixels whose world z lies in [zmin,zmax]."""
    H, W = depth.shape
    vv, uu = np.mgrid[0:H, 0:W]
    z = depth
    ok = np.isfinite(z) & (z > 0.02)
    if roi is not None:
        u0, v0, u1, v1 = roi
        ok &= (uu >= u0) & (uu < u1) & (vv >= v0) & (vv < v1)
    pc = np.stack([(uu - K[0, 2]) * z / K[0, 0], (vv - K[1, 2]) * z / K[1, 1],
                   z, np.ones_like(z)], -1)
    pw = pc @ T.T
    sel = ok & (pw[..., 2] > zmin) & (pw[..., 2] < zmax)
    pts = pw[sel][:, :3]
    if tag and bgr is not None:
        vis = bgr.copy(); vis[sel] = (0, 255, 0); cv2.imwrite(f"{tag}_rim.png", vis)
    return pts
