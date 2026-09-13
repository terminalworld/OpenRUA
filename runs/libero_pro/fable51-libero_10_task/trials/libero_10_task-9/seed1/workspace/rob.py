#!/usr/bin/env python3
"""Reusable robot helpers: joint state, FK/IK (MoveIt), trajectories, gripper.

World frame = panda_link0 + (-0.66, 0, 0.912) (from TF). MoveIt poses are in
panda_link0 (frame_id left empty as machine.yaml says).
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
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM_JOINTS = FJT["joints"]
# Verified empirically: /compute_fk and /compute_ik here operate directly in
# the world frame (FK of the hand matches the camera-observed hand position),
# so no base offset is applied.
BASE_IN_WORLD = np.zeros(3)
TCP_OFF = 0.1034


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t = time.time()
        while "m" not in self._js and time.time() - t < 10:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM_JOINTS]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        """World pose (pos, quat xyzw) of link for arm config q."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM_JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = Rot.from_quat(quat).as_matrix()
        return pos + TCP_OFF * R[:, 2], quat

    # ---------- planning ----------
    def ik(self, pos_world, quat_xyzw, seed=None, link="panda_hand", at_tcp=True,
           timeout=5.0, attempts=1):
        """IK for the hand (or TCP if at_tcp) at a world pose. Returns q or None."""
        pos = np.asarray(pos_world, float)
        R = Rot.from_quat(quat_xyzw).as_matrix()
        if at_tcp:
            pos = pos - TCP_OFF * R[:, 2]
        pos = pos - BASE_IN_WORLD
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = link
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat_xyzw)
        req.ik_request.robot_state.joint_state.name = list(ARM_JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        for _ in range(attempts):
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM_JOINTS]
        return None

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, wait=True):
        return self.move_traj([q], [seconds], wait)

    def move_traj(self, qs, times, wait=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        for q, t in zip(qs, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if not wait:
            return gh
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)
        code = rf.result().result.error_code
        q_now = self.arm_q()
        err = float(np.max(np.abs(np.array(q_now) - np.array(qs[-1]))))
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_q_corrected(self, q_target, seconds=3.0, iters=3, tol=0.006, tol7=0.03,
                         j7_rate=0.15):
        """Move to q_target, then re-send with residual compensation.

        Machine facts observed: joint 7 tracks at only ~0.2 rad/s, and joints
        under gravity load settle ~0.01-0.02 rad short of the command. So the
        command is offset by the measured residual and joint 7 gets extra time.
        """
        q_target = np.asarray(q_target, float)
        offset = np.zeros(7)
        q_now = np.asarray(self.arm_q())
        for i in range(iters + 1):
            d7 = abs(q_target[6] + offset[6] - q_now[6])
            dur = max(seconds if i == 0 else 2.0, d7 / j7_rate + 0.5)
            self.move_q(list(q_target + offset), dur)
            q_now = np.asarray(self.arm_q())
            resid = q_target - q_now
            print(f"  iter{i} resid={np.round(resid, 4)}")
            if np.max(np.abs(resid[:6])) < tol and abs(resid[6]) < tol7:
                break
            offset[:6] += resid[:6]
            offset[6] = 0.0  # joint 7 has no sag, only lag
        return q_now

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f


def quat_from_axes(x_h, y_h, z_h):
    """Quaternion (xyzw) of a frame whose axes (in world) are given as columns."""
    R = np.column_stack([x_h, y_h, z_h])
    return Rot.from_matrix(R).as_quat()


def topdown_quat(finger_axis_world):
    """Hand pointing straight down, fingers closing along finger_axis_world."""
    y_h = np.asarray(finger_axis_world, float)
    y_h = y_h / np.linalg.norm(y_h)
    z_h = np.array([0, 0, -1.0])
    x_h = np.cross(y_h, z_h)
    return quat_from_axes(x_h, y_h, z_h)


def approach_quat(approach_world, finger_axis_world):
    """Hand z along approach, fingers along finger_axis (must be perpendicular)."""
    z_h = np.asarray(approach_world, float); z_h /= np.linalg.norm(z_h)
    y_h = np.asarray(finger_axis_world, float)
    y_h = y_h - np.dot(y_h, z_h) * z_h; y_h /= np.linalg.norm(y_h)
    x_h = np.cross(y_h, z_h)
    return quat_from_axes(x_h, y_h, z_h)
