#!/usr/bin/env python3
"""Reusable arm helpers: joint state, IK (world-frame pose), trajectory, gripper.

World -> planning frame (panda_link0) offset comes from TF at import time.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = M["hand"]["tcp_offset_m"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # panda_link0 in world (TF)
HOME = [0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854]  # initial config

# quaternion: hand pointing down, fingers open along world y
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)
# hand pointing down, fingers open along world x (rotated 90 deg about z)
Q_DOWN_X = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self, name="arm_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.fjt.wait_for_server(timeout_sec=20)
        self.grip.wait_for_server(timeout_sec=20)
        self.ik.wait_for_service(timeout_sec=20)

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))

    def spin(self, t=0.2):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self._js = {}
        while not self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in JOINTS]

    def finger_gap(self):
        js = self.joints()
        return js["panda_finger_joint1"], js["panda_finger_joint2"]

    def hand_pose_world(self):
        """TF panda_link0->panda_hand, shifted into world."""
        self.spin(0.3)
        t = self.tfbuf.lookup_transform("panda_link0", "panda_hand", rclpy.time.Time())
        tr = t.transform.translation
        q = t.transform.rotation
        p = np.array([tr.x, tr.y, tr.z]) + BASE_IN_WORLD
        return p, (q.x, q.y, q.z, q.w)

    def tcp_world(self):
        p, q = self.hand_pose_world()
        R = quat_R(*q)
        return p + TCP * R[:, 2], q

    def ik_world(self, xyz_tcp, quat, seed=None, at_tcp=True, tries=3):
        """IK for a world-frame TCP pose. Returns joint list or None."""
        xyz = np.array(xyz_tcp, dtype=float)
        if at_tcp:
            R = quat_R(*quat)
            xyz = xyz - TCP * R[:, 2]
        xyz_base = xyz  # planner model frame == world on this machine (verified via FK)
        seed = seed if seed is not None else self.arm_q()
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            # the group's default tip is panda_link8 (45 deg off panda_hand
            # about z) -> solve for the hand link explicitly
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, xyz_base)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            s = JointState()
            s.name = list(JOINTS)
            s.position = [float(v) for v in seed]
            req.ik_request.robot_state.joint_state = s
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
            # perturb seed and retry
            seed = [v + np.random.uniform(-0.3, 0.3) for v in seed]
        return None

    def ik_best(self, xyz_tcp, quat, n=6):
        """Several IK calls (current + home + perturbed seeds); pick the
        solution nearest the current joints."""
        cur = np.array(self.arm_q())
        seeds = [cur, np.array(HOME)] + [cur + np.random.uniform(-0.4, 0.4, 7) for _ in range(n - 2)]
        best = None
        for s in seeds:
            sol = self.ik_world(xyz_tcp, quat, seed=list(s), tries=1)
            if sol is None:
                continue
            d = np.abs(np.array(sol) - cur).max()
            if best is None or d < best[0]:
                best = (d, sol)
        return None if best is None else best[1]

    def move_joints(self, q, secs=3.0, via=None, _retry=True):
        delta = np.abs(np.array(q) - np.array(self.arm_q())).max()
        secs = max(secs, float(delta) / 0.4)   # <= 0.4 rad/s on the fastest joint
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                t = secs * (i + 1) / n
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=120)
        gh = send.result()
        if gh is None or not gh.accepted:
            print("FJT goal rejected", flush=True)
            return None
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=900)
        if res.result() is None:
            print("FJT no result (timeout)", flush=True)
            return None
        code = res.result().result.error_code
        self.spin(0.3)
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"FJT code={code} max_joint_err={err:.4f} (secs={secs:.1f})", flush=True)
        if err > 0.01 and _retry:
            print("  -> re-sending goal", flush=True)
            nxt = 2 if _retry is True else _retry - 1   # up to 3 sends total
            return self.move_joints(q, secs=max(3.0, secs), _retry=nxt if nxt > 0 else False)
        return code

    def move_tcp(self, xyz, quat, secs=3.0, seed=None):
        q = self.ik_best(xyz, quat) if seed is None else self.ik_world(xyz, quat, seed=seed)
        if q is None:
            print(f"IK FAILED for {xyz}", flush=True)
            return None
        code = self.move_joints(q, secs)
        p, _ = self.tcp_world()
        print(f"tcp now {p.round(4)} target {np.array(xyz).round(4)}", flush=True)
        return q

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result().result if res.result() else None
        g = self.finger_gap()
        print(f"gripper -> {width}: reached={getattr(r,'reached_goal',None)} "
              f"stalled={getattr(r,'stalled',None)} fingers={g}", flush=True)
        return g

    def open(self):
        return self.gripper(GRIP["open_m"])

    def close(self):
        return self.gripper(GRIP["closed_m"])

    def move_line(self, xyz_to, quat, secs=3.0, steps=4, seed=None):
        """Straight-ish TCP line: IK at intermediate points, one trajectory."""
        p0, _ = self.tcp_world()
        p1 = np.array(xyz_to, dtype=float)
        seed = seed if seed is not None else self.arm_q()
        via = []
        for i in range(1, steps + 1):
            pt = p0 + (p1 - p0) * i / steps
            q = self.ik_world(pt, quat, seed=seed)
            if q is None:
                print(f"IK FAILED on line at {pt}", flush=True)
                return None
            via.append(q)
            seed = q
        code = self.move_joints(via[-1], secs, via=via[:-1])
        p, _ = self.tcp_world()
        print(f"tcp now {p.round(4)} target {p1.round(4)}", flush=True)
        return via[-1]


def R_quat(R):
    """Rotation matrix -> quaternion (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def rot_axis(axis, ang):
    """Rodrigues rotation matrix about unit axis by ang (rad)."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def _move_path(self, poses, secs=4.0, seed=None):
    """Sequential IK through a list of (xyz_tcp, quat); one trajectory."""
    seed = seed if seed is not None else self.arm_q()
    via = []
    for xyz, quat in poses:
        q = self.ik_world(xyz, quat, seed=seed)
        if q is None:
            print(f"IK FAILED on path at {np.round(xyz, 3)}", flush=True)
            return None
        if np.abs(np.array(q) - np.array(seed)).max() > 1.2:
            print(f"IK branch jump on path at {np.round(xyz, 3)}", flush=True)
            return None
        via.append(q)
        seed = q
    self.move_joints(via[-1], secs, via=via[:-1])
    p, qq = self.tcp_world()
    print(f"tcp now {p.round(4)} quat {np.round(qq, 3)}", flush=True)
    return via[-1]


Arm.move_path = _move_path
