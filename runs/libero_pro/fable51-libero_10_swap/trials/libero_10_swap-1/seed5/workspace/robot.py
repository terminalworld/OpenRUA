#!/usr/bin/env python3
"""Reusable controller for this Panda: FK/IK, trajectory, gripper, sensing.

World frame poses are converted to the arm base (panda_link0) for MoveIt
(world -> base offset read from /tf).
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Image
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])

# top-down grasp: hand z -> -world z, hand x -> +world x (fingers along world y)
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def qmul(q1, q2):
    """Hamilton product, quaternions as (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


# verified: /compute_ik solves for panda_link8, which is rotated -45 deg
# about z from panda_hand (tf_static). hand = link8 * Rz(-45) => link8 = hand * Rz(+45)
Q_HAND_TO_LINK8 = (0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8))

# joint7 tracks at most ~0.19 rad/s on this machine; pace moves by it
MAX_JOINT_VEL = 0.15


def yaw_down_quat(yaw):
    """Top-down grasp with fingers rotated by `yaw` (rad) about world z
    from the Q_DOWN configuration (fingers along world y)."""
    # q = qz(yaw) * (1,0,0,0)
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    # qz = (0,0,s,c); qx = (1,0,0,0); product qz*qx:
    # w = c*0 - s*0 = 0? compute properly: (w1w2 - v1.v2, w1v2 + w2v1 + v1xv2)
    w1, v1 = c, np.array([0, 0, s])
    w2, v2 = 0.0, np.array([1.0, 0, 0])
    w = w1 * w2 - v1 @ v2
    v = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    return (v[0], v[1], v[2], w)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_ctl")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(30)
        self.grip.wait_for_server(30)
        self.ik.wait_for_service(30)
        self.fk.wait_for_service(30)
        # verified: MoveIt's model frame is `world` (panda_link0 at
        # (-0.51,0,0.42) inside it), so empty-frame poses ARE world poses
        self.base_off = np.zeros(3)
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.5)

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == "world" and t.child_frame_id == "panda_link0":
                tr = t.transform.translation
                self.base_off = np.array([tr.x, tr.y, tr.z])

    def _on_js(self, msg):
        self.js = dict(zip(msg.name, msg.position))

    def spin(self, n=5):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def joints(self):
        self.js = None
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.5)
        return [self.js[j] for j in ARM]

    def fingers(self):
        self.joints()
        return self.js.get("panda_finger_joint1"), self.js.get("panda_finger_joint2")

    def _arm_state(self):
        st = JointState()
        st.name = list(ARM)
        st.position = self.joints()
        return st

    def fk_hand(self, positions=None):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        st = JointState()
        st.name = list(ARM)
        st.position = positions if positions is not None else self.joints()
        req.robot_state.joint_state = st
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + self.base_off
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), res.error_code.val

    def ik_world(self, xyz, quat, at_tcp=True, seed=None):
        """IK for the hand (or TCP) at a world-frame pose. Returns joint list or None."""
        xyz = np.array(xyz, dtype=float)
        if at_tcp:
            R = quat_to_R(*quat)
            xyz = xyz - TCP * R[:, 2]
        base = xyz - self.base_off
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = base.tolist()
        q8 = qmul(quat, Q_HAND_TO_LINK8)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q8
        st = JointState()
        st.name = list(ARM)
        st.position = list(seed) if seed is not None else self.joints()
        req.ik_request.robot_state.joint_state = st
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK failed: {None if res is None else res.error_code.val}")
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        sol = [sol[j] for j in ARM]
        # verify the hand pose of the solution against the request
        pos, q, _ = self.fk_hand(sol)
        R = quat_to_R(*q)
        Rt = quat_to_R(*quat)
        ang = np.degrees(np.arccos(np.clip((np.trace(R.T @ Rt) - 1) / 2, -1, 1)))
        perr = np.linalg.norm(pos - xyz)
        if perr > 0.005 or ang > 3:
            print(f"IK solution mismatch: pos err {perr:.4f} m, ang err {ang:.1f} deg")
            return None
        return sol

    def move_joints(self, positions, seconds=3.0, via=None):
        delta = np.abs(np.array(positions) - np.array(self.joints())).max()
        seconds = max(seconds, float(delta) / MAX_JOINT_VEL)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                t = seconds * (i + 1) / n
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        t0 = time.time()
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=120)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=900)
        code = res.result().result.error_code if res.result() else None
        cur = np.array(self.joints())
        err = np.abs(cur - np.array(positions)).max()
        print(f"traj done code={code} max_joint_err={err:.4f} ({seconds:.1f}s traj, {time.time()-t0:.0f}s wall)")
        return code, err

    def move_world(self, xyz, quat=Q_DOWN, seconds=3.0, at_tcp=True, seed=None):
        sol = self.ik_world(xyz, quat, at_tcp=at_tcp, seed=seed)
        if sol is None:
            return None
        code, err = self.move_joints(sol, seconds)
        pos, q, _ = self.fk_hand()
        R = quat_to_R(*q)
        tcp = pos + TCP * R[:, 2]
        print(f"  hand at {np.round(pos,4)} tcp at {np.round(tcp,4)} (target {np.round(xyz,4)})")
        return sol

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result if res.result() else None
        f = self.fingers()
        print(f"gripper -> {width}: reached={getattr(r,'reached_goal',None)} stalled={getattr(r,'stalled',None)} fingers={f}")
        return f

    def snap(self, cam, out=None):
        from cv_bridge import CvBridge
        import cv2
        got = {}
        sub = self.node.create_subscription(Image, f"/{cam}/color/image_raw",
                                            lambda m: got.setdefault("m", m), 1)
        end = time.time() + 60
        while "m" not in got and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.node.destroy_subscription(sub)
        out = out or f"{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got["m"], "bgr8"))
        return out
