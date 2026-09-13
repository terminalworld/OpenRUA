#!/usr/bin/env python3
"""Small helper around IK / FJT / gripper for this Panda (machine.yaml facts).

import arm; a = arm.Arm()
a.js()                      -> dict joint->pos
a.fk()                      -> (xyz, quat) of panda_hand in world
a.ik(xyz, quat, at_tcp=True, seed=None) -> list of 7 joint positions or None
a.move_joints([q1, q2,...], seconds_per_seg) -> error_code
a.move_poses([(xyz,quat),...], seconds_per_seg, at_tcp=True) -> error_code
a.grip(width_per_finger)    -> (reached, stalled)
"""
import sys
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

ROOT = Path(__file__).resolve().parent
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
PLAN = M["planning"]
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP = float(M["hand"]["tcp_offset_m"])
RZ45 = np.array([[np.cos(np.pi/4), -np.sin(np.pi/4), 0], [np.sin(np.pi/4), np.cos(np.pi/4), 0], [0, 0, 1]])
BASE = np.array([0.0, 0.0, 0.0])  # FK/IK poses come back in world coords (verified vs TF)


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    # returns (x, y, z, w)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def quat_from_axes(xaxis, yaxis, zaxis):
    R = np.stack([np.asarray(xaxis, float), np.asarray(yaxis, float),
                  np.asarray(zaxis, float)], axis=1)
    return R_to_quat(R)


class Arm:
    def __init__(self, name="arm_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.ik_cli = self.node.create_client(GetPositionIK, PLAN["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)
        self.fjt.wait_for_server(10)
        self.gr.wait_for_server(10)

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def js(self, fresh=True):
        if fresh:
            self._js = {}
        while not self._js:
            self.spin(0.2)
        return dict(self._js)

    def arm_q(self):
        j = self.js()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.js()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def _seed(self, q):
        s = JointState()
        s.name = list(JOINTS)
        s.position = [float(v) for v in q]
        return s

    def fk(self, q=None):
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z,
                         p.orientation.w])
        return xyz, quat

    def ik(self, xyz, quat, at_tcp=True, seed=None, timeout=20.0):
        xyz = np.asarray(xyz, float)
        quat = np.asarray(quat, float)
        if at_tcp:
            xyz = xyz - TCP * quat_to_R(quat)[:, 2]
        xyz = xyz - BASE  # planner frame is the arm base
        # the IK tip is panda_link8; panda_hand = link8 rotated -45deg about Z
        # (verified vs TF), so request R_link8 = R_hand * Rz(+45deg)
        quat = R_to_quat(quat_to_R(quat) @ RZ45)
        req = GetPositionIK.Request()
        req.ik_request.group_name = PLAN["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        (p.orientation.x, p.orientation.y, p.orientation.z,
         p.orientation.w) = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(
            seed if seed is not None else self.arm_q())
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout % 1) * 1e9)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            print("IK: no answer", file=sys.stderr)
            return None
        if res.error_code.val != 1:
            print(f"IK failed code={res.error_code.val}", file=sys.stderr)
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        q = [sol[j] for j in JOINTS]
        for i, (v, (lo, hi)) in enumerate(zip(q, LIMITS)):
            if v < lo or v > hi:
                print(f"IK: joint{i+1}={v:.3f} outside [{lo},{hi}]",
                      file=sys.stderr)
        return q

    def move_joints(self, qs, seconds_per_seg=3.0, first_seconds=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        t = 0.0
        for i, q in enumerate(qs):
            dt = first_seconds if (i == 0 and first_seconds) else seconds_per_seg
            t += dt
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        gh = send.result()
        if not gh.accepted:
            print("FJT goal rejected", file=sys.stderr)
            return -100
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf)
        code = rf.result().result.error_code
        q_now = np.array(self.arm_q())
        err = np.abs(q_now - np.array(qs[-1])).max()
        print(f"FJT done code={code} max_joint_err={err:.4f}")
        return code

    def move_poses(self, poses, seconds_per_seg=3.0, at_tcp=True,
                   first_seconds=None):
        qs = []
        seed = self.arm_q()
        for xyz, quat in poses:
            q = self.ik(xyz, quat, at_tcp=at_tcp, seed=seed)
            if q is None:
                print(f"IK failed for {np.round(xyz,3)}; no motion",
                      file=sys.stderr)
                return None
            qs.append(q)
            seed = q
        return self.move_joints(qs, seconds_per_seg, first_seconds)

    def grip(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.fingers()
        print(f"grip reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return r.reached_goal, r.stalled


# handy orientations (x,y,z,w), hand Z = approach, hand Y = finger axis
DOWN = np.array([1.0, 0.0, 0.0, 0.0])            # Z down, fingers along world y
DOWN_FX = quat_from_axes([0, 1, 0], [1, 0, 0], [0, 0, -1])   # Z down, fingers along x
FWD_Y = quat_from_axes([0, 0, 1], [1, 0, 0], [0, 1, 0])      # Z -> +y, fingers along x, camera up


def tilt_y(deg, fingers_x=True):
    """Hand Z pointing +y and rotated `deg` downward; fingers along world x."""
    s, c = np.sin(np.radians(deg)), np.cos(np.radians(deg))
    Z = np.array([0.0, c, -s])
    Y = np.array([1.0, 0, 0]) if fingers_x else np.array([-1.0, 0, 0])
    X = np.cross(Y, Z)
    return quat_from_axes(X, Y, Z)


def chain_ik(a, poses, at_tcp=True, seed=None, max_jump=1.6):
    """IK for a list of (xyz, quat), each seeded with the previous solution.
    Returns list of q or None; prints per-segment max joint jump."""
    qs = []
    seed = a.arm_q() if seed is None else list(seed)
    for i, (xyz, quat) in enumerate(poses):
        q = None
        for _ in range(3):
            q = a.ik(xyz, quat, at_tcp=at_tcp, seed=seed)
            if q is not None and np.abs(np.array(q) - np.array(seed)).max() <= max_jump:
                break
        if q is None:
            print(f"chain_ik: no IK for wp{i} {np.round(xyz,3)}")
            return None
        jump = np.abs(np.array(q) - np.array(seed)).max()
        print(f"wp{i} {np.round(xyz,3)} jump={jump:.2f} q={np.round(q,3)}")
        if jump > max_jump:
            print("chain_ik: jump too large, aborting")
            return None
        qs.append(q)
        seed = q
    return qs


# --- crude collision check: sample joint-space segments, FK several links ---
LINKS = ["panda_link4", "panda_link5", "panda_link6", "panda_link7", "panda_hand"]
# boxes as (name, xmin,xmax, ymin,ymax, zmin,zmax) already inflated for link radius
OBST = [
    ("microwave", -0.36, 0.15, -0.43, -0.08, 0.0, 1.17),
    ("door", -0.38, -0.22, -0.68, -0.36, 0.0, 1.17),
    ("table", -0.80, 0.56, -0.66, 0.66, 0.0, 0.96),
    ("grey_mug", -0.09, 0.11, 0.24, 0.44, 0.0, 1.06),
]
YELLOW_MUG = ("yellow_mug", -0.09, 0.11, -0.09, 0.11, 0.0, 1.06)


def fk_links(a, q, links=LINKS):
    req = GetPositionFK.Request()
    req.header.frame_id = ""
    req.fk_link_names = list(links)
    req.robot_state.joint_state = a._seed(q)
    fut = a.fk_cli.call_async(req)
    rclpy.spin_until_future_complete(a.node, fut, timeout_sec=30)
    res = fut.result()
    out = {}
    for name, ps in zip(res.fk_link_names, res.pose_stamped):
        p = ps.pose
        xyz = np.array([p.position.x, p.position.y, p.position.z])
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        out[name] = (xyz, quat)
    if "panda_hand" in out:
        h, hq = out["panda_hand"]
        out["tcp"] = (h + TCP * quat_to_R(hq)[:, 2], hq)
        out["fingers_mid"] = (h + 0.07 * quat_to_R(hq)[:, 2], hq)
    return out


def in_box(p, b):
    return b[1] <= p[0] <= b[2] and b[3] <= p[1] <= b[4] and b[5] <= p[2] <= b[6]


def check_path(a, q_start, qs, n=8, obst=None, allow=()):
    """Sample linear joint interpolation; report link points inside obstacle
    boxes. `allow` = set of (link, obstacle) pairs to ignore (e.g. tcp near
    table when grasping). Returns list of violations."""
    obst = OBST if obst is None else obst
    viol = []
    prev = np.array(q_start, float)
    for si, q in enumerate(qs):
        q = np.array(q, float)
        for k in range(1, n + 1):
            qi = prev + (q - prev) * k / n
            L = fk_links(a, qi)
            for name, (p, _) in L.items():
                for b in obst:
                    if (name, b[0]) in allow:
                        continue
                    # links above the flange only checked against tall obstacles
                    if in_box(p, b):
                        viol.append((si, k, name, b[0], np.round(p, 3)))
        prev = q
    return viol
