#!/usr/bin/env python3
"""Reusable helpers: FK/IK in world frame, trajectory, gripper, joint read.
World <-> base: base (panda_link0) sits at world (-0.51, 0, 0.42), no rotation.
"""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE = np.array([0.0, 0.0, 0.0])  # MoveIt model frame == world (verified by FK vs camera TF)
TCP = M["hand"]["tcp_offset_m"]
# hand pointing down, fingers closing along world Y
Q_FY = (0.0, 1.0, 0.0, 0.0)
# hand pointing down, fingers closing along world X
Q_FX = (0.70710678, 0.70710678, 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def qmul(a, b):
    """Hamilton product of (x,y,z,w) quaternions."""
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)


# MoveIt's panda_arm tip is panda_link8; panda_hand = link8 * Rz(-45deg).
# So a desired HAND orientation must be sent as link8 = hand * Rz(+45deg).
Q_HAND_TO_LINK8 = (0.0, 0.0, float(np.sin(np.pi / 8)), float(np.cos(np.pi / 8)))


def yaw_quat(yaw):
    """Hand down, fingers closing along world direction rotated `yaw` from +Y."""
    # R = Rz(yaw) @ Ry(pi)
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    # q = qz(yaw) * qy(pi) ; qy(pi) = (0,1,0,0)
    # (w1,v1)*(w2,v2) = (w1w2 - v1.v2, w1 v2 + w2 v1 + v1 x v2)
    w1, v1 = c, np.array([0, 0, s]); w2, v2 = 0.0, np.array([0, 1.0, 0])
    w = w1 * w2 - v1 @ v2
    v = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    return (float(v[0]), float(v[1]), float(v[2]), float(w))


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def fk(self, q=None):
        """World-frame hand pose (pos, quat) and TCP pos."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(c) for c in (q if q is not None else self.arm_q())]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q4 = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        tcp = pos + TCP * quat_R(*q4)[:, 2]
        return pos, q4, tcp

    def ik(self, tcp_xyz, quat, seed=None, tries=5):
        """IK for a world-frame TCP position + hand orientation -> joint list."""
        R = quat_R(*quat)
        hand = np.array(tcp_xyz, float) - TCP * R[:, 2] - BASE
        seed = seed if seed is not None else self.arm_q()
        quat = qmul(quat, Q_HAND_TO_LINK8)  # hand orientation -> link8 orientation
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = hand
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = [float(c) for c in quat]
            req.ik_request.robot_state.joint_state.name = JOINTS
            req.ik_request.robot_state.joint_state.position = [float(c) for c in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
            print("IK attempt failed", None if res is None else res.error_code.val)
        return None

    def move_q(self, q, seconds=3.0, via=None, retries=2):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = seconds * (i + 1) / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  traj error_code={code} max_joint_err={err:.4f}")
        if err > 0.01 and retries > 0:
            print("  re-sending goal to converge")
            return self.move_q(q, max(2.0, seconds / 2), retries=retries - 1)
        return code, err

    def move_tcp(self, tcp_xyz, quat, seconds=3.0, seed=None):
        q = self.ik(tcp_xyz, quat, seed=seed)
        if q is None:
            print("  IK FAILED for", tcp_xyz)
            return None
        code, err = self.move_q(q, seconds)
        pos, _, tcp = self.fk()
        print(f"  tcp now {tcp.round(4)} (target {np.round(tcp_xyz,4)})")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f


READY = [0.0, -0.5, 0.0, -2.2, 0.0, 1.7, 0.785]


def good_ik(r, tcp_xyz, yaws=(0.0, np.pi, np.pi / 2, -np.pi / 2), fixed_quat=None):
    """Try several yaws/seeds; return (q, quat) closest to the current config
    with a sane base-joint azimuth."""
    cur = np.array(r.arm_q())
    az = np.arctan2(tcp_xyz[1] - 0.0, tcp_xyz[0] + 0.51)
    seeds = [list(cur), [az] + READY[1:]]
    quats = [fixed_quat] if fixed_quat is not None else [yaw_quat(y) for y in yaws]
    best = None
    for quat in quats:
        for s in seeds:
            q = r.ik(tcp_xyz, quat, seed=s, tries=2)
            if q is None:
                continue
            q = np.array(q)
            if abs(q[0] - az) > 1.2:  # avoid wrapped-around base solutions
                continue
            d = np.abs(q - cur).sum()
            if best is None or d < best[0]:
                best = (d, q, quat)
    return (best[1], best[2]) if best else (None, None)
