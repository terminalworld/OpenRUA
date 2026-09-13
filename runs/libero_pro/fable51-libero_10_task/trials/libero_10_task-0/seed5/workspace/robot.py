#!/usr/bin/env python3
"""Reusable clients: joint state, IK, FK, trajectory, gripper. World<->base."""
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load((Path(__file__).parent / "machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
# Verified empirically: /compute_fk and /compute_ik on this machine take and
# return poses in the WORLD frame (analytic Panda FK of the current config
# matched the service output only after adding /tf world->panda_link0).
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])
TCP = float(M["hand"]["tcp_offset_m"])
TOP_DOWN = (1.0, 0.0, 0.0, 0.0)  # hand z down, fingers close along world y


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    """Hamilton product a*b, quaternions as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def yaw_quat(yaw):
    """Top-down hand orientation rotated by yaw about world z."""
    # q = Rz(yaw) * Rx(pi)
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    # Rz(yaw) = (0,0,s,c); Rx(pi) = (1,0,0,0); product (w1w2 - v1.v2, ...)
    # q1=(x=0,y=0,z=s,w=c), q2=(1,0,0,0)
    x = c * 1 + 0 + (0 * 0 - s * 0)
    y = 0 + 0 + (s * 1 - 0 * 0)
    z = c * 0 + s * 0 + (0 * 0 - 0 * 1)
    w = c * 0 - (0 * 1 + 0 + s * 0)
    # simplified: (x,y,z,w) = (c, s, 0, 0)
    return (float(c), float(s), 0.0, 0.0)


class Robot:
    def __init__(self, name="robot_ctl"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.traj.wait_for_server(10)
        self.grip.wait_for_server(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        end = time.time() + 15
        while "m" not in self._js and time.time() < end:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def solve_ik(self, pos_world, quat, seed=None, at_tcp=True, timeout=60):
        pos = np.array(pos_world, dtype=float)
        if at_tcp:
            R = quat_to_R(*quat)
            pos = pos - TCP * R[:, 2]
        pb = pos - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pb)
        # /compute_ik solves for panda_link8 (verified via /compute_fk), which
        # is panda_hand rotated +45 deg about its z: q_link8 = q_hand * Rz(pi/4)
        q8 = quat_mul(quat, (0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)))
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)
        seed_q = seed if seed is not None else self.arm_q()
        js = JointState()
        js.name = list(ARM)
        js.position = [float(v) for v in seed_q]
        req.ik_request.robot_state.joint_state = js
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            raise RuntimeError(f"IK failed code={res.error_code.val} for world {pos_world}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in ARM]

    def fk_hand(self, q=None):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        js = JointState()
        js.name = list(ARM)
        js.position = [float(v) for v in (q if q is not None else self.arm_q())]
        req.robot_state.joint_state = js
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, quat

    def tcp_world(self, q=None):
        pos, quat = self.fk_hand(q)
        return pos + TCP * quat_to_R(*quat)[:, 2], quat

    def settle(self, max_reads=40):
        """Re-read joints until two consecutive reads agree (controller lag)."""
        prev = np.array(self.arm_q())
        for _ in range(max_reads):
            cur = np.array(self.arm_q())
            if np.abs(cur - prev).max() < 1e-4:
                return cur
            prev = cur
        return prev

    def move(self, q, seconds=3.0, waypoints=None, tol=0.01, retries=2):
        for attempt in range(retries + 1):
            code, err = self._move_once(q, seconds, waypoints)
            if err <= tol:
                break
            print(f"  move: retry {attempt + 1} (err={err:.4f})", flush=True)
            waypoints = None
        return code, err

    def _move_once(self, q, seconds, waypoints):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = list(waypoints or []) + [q]
        for i, wq in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(v) for v in wq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(self.settle() - np.array(q)).max()
        print(f"  move: code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        gap = self.finger_gap()
        for _ in range(40):  # settle ticks
            g2 = self.finger_gap()
            if abs(g2 - gap) < 1e-5:
                break
            gap = g2
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap

    def goto(self, pos_world, quat=TOP_DOWN, seconds=3.0, seed=None):
        q = self.solve_ik(pos_world, quat, seed=seed)
        code, err = self.move(q, seconds)
        tcp, _ = self.tcp_world()
        print(f"  tcp now {tcp.round(4)} target {np.array(pos_world).round(4)}", flush=True)
        return q
