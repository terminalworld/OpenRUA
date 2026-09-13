#!/usr/bin/env python3
"""Reusable robot helpers: joint state, IK, FK, trajectory, gripper, TF."""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from rclpy.time import Time
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = None


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self.node)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)
        self.spin(0.5)

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------- sensing ----------
    def js(self):
        self._js.pop("m", None)
        while "m" not in self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.js(); return [j[n] for n in ARM]

    def fingers(self):
        j = self.js()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def tf(self, a, b, tries=50):
        for _ in range(tries):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.tfbuf.can_transform(a, b, Time()):
                t = self.tfbuf.lookup_transform(a, b, Time())
                tr, q = t.transform.translation, t.transform.rotation
                return np.array([tr.x, tr.y, tr.z]), np.array([q.x, q.y, q.z, q.w])
        raise RuntimeError(f"no tf {a}->{b}")

    def base_in_world(self):
        global BASE_IN_WORLD
        if BASE_IN_WORLD is None:
            BASE_IN_WORLD = self.tf("world", M["frames"]["base"])[0]
        return BASE_IN_WORLD

    def hand_pose_world(self, q=None):
        """FK of the hand frame, in the world frame (base offset added)."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [M["frames"]["hand"]]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = q if q is not None else self.arm_q()
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])  # already world
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def tcp_pose_world(self, q=None):
        pos, quat = self.hand_pose_world(q)
        R = Rot.from_quat(quat).as_matrix()
        return pos + TCP_OFF * R[:, 2], quat

    # ---------- IK ----------
    def ik(self, pos_world, quat, at_tcp=True, seed=None, timeout=60):
        """pos_world: target in world; quat xyzw of hand. Returns arm joints or None."""
        pos = np.array(pos_world, float)
        R = Rot.from_quat(quat).as_matrix()
        if at_tcp:
            pos = pos - TCP_OFF * R[:, 2]
        # machine fact (verified): this planner's model frame IS world
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = M["frames"]["hand"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(seed) if seed is not None else self.arm_q()
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        r = fut.result()
        if r is None:
            print("IK: no answer"); return None
        if r.error_code.val != 1:
            print(f"IK failed code={r.error_code.val}"); return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    # ---------- acting ----------
    def move_joints(self, q_list, secs, verbose=True):
        """q_list: list of joint vectors (waypoints); secs: total or per-point list."""
        if not isinstance(q_list[0], (list, tuple, np.ndarray)):
            q_list = [q_list]
        if not isinstance(secs, (list, tuple)):
            n = len(q_list); secs = [secs * (i + 1) / n for i in range(n)]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        for q, t in zip(q_list, secs):
            pt = JointTrajectoryPoint(positions=[float(x) for x in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q_list[-1])).max()
        if verbose:
            print(f"traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos_world, quat, secs=3.0, seed=None):
        q = self.ik(pos_world, quat, at_tcp=True, seed=seed)
        if q is None:
            return None
        code, err = self.move_joints(q, secs)
        p, _ = self.tcp_pose_world()
        print(f"  tcp now {np.round(p,4)} target {np.round(pos_world,4)}")
        return q

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        f = self.fingers()
        print(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f

    def shutdown(self):
        self.node.destroy_node(); rclpy.shutdown()


def quat_from_axes(hand_x, hand_y, hand_z):
    R = np.column_stack([hand_x, hand_y, hand_z])
    return Rot.from_matrix(R).as_quat()  # xyzw


Q_DOWN = np.array([1.0, 0.0, 0.0, 0.0])  # hand z down, fingers along world y
