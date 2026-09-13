#!/usr/bin/env python3
"""Small motion library for this Panda: IK, trajectories, gripper, FK.

World frame <-> panda_link0: base sits at world (-0.51, 0, 0.42), identity
rotation (from TF). MoveIt plans in panda_link0, so world poses are
shifted by BASE before IK.
"""
import math
import sys
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

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
# Verified: /compute_fk and /compute_ik on this machine work in the WORLD
# frame (FK of link0 returns (-0.51,0,0.42)), so no base offset is applied.
BASE = np.array([0.0, 0.0, 0.0])
TCP = M["hand"]["tcp_offset_m"]
RATE = 0.15  # rad/s the controller seems to sustain; used to size durations

# top-down grasp orientations (hand z down). yaw = rotation about world z
# of the finger axis: yaw 0 -> fingers along world y, yaw 90deg -> along x.
def down_quat(yaw_deg=0.0):
    # q = Rz(yaw) * Rx(180)
    h = math.radians(yaw_deg) / 2
    qz = np.array([0, 0, math.sin(h), math.cos(h)])
    qx = np.array([1, 0, 0, 0])
    return quat_mul(qz, qx)


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_lib")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(timeout_sec=20), "no FJT server"
        assert self.grip.wait_for_server(timeout_sec=20), "no gripper server"
        assert self.ik_cli.wait_for_service(timeout_sec=20), "no IK"
        assert self.fk_cli.wait_for_service(timeout_sec=20), "no FK"

    def _on_js(self, msg):
        self._js = msg

    # ---------- sensing ----------
    def joints(self, fresh=True):
        """Arm joint positions in manifest order (dict also has fingers)."""
        if fresh:
            self._js = None
        t0 = time.time()
        while self._js is None and time.time() - t0 < 30:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return np.array([d[j] for j in JOINTS]), d

    def finger_gap(self):
        _, d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        """World pose (xyz, quat) of a link for arm joints q (default current)."""
        if q is None:
            q, _ = self.joints()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return xyz, q

    def tcp(self, q=None):
        """World position of the fingertip centre (TCP)."""
        xyz, quat = self.fk(q)
        return xyz + TCP * quat_R(quat)[:, 2], quat

    # ---------- planning ----------
    def ik(self, xyz_world, quat, seed=None, at_tcp=True, tries=8, max_dist=1.5):
        """IK for a world pose of the HAND (or TCP if at_tcp). Returns the
        solution closest (in joint space) to seed, or None. max_dist is the
        largest acceptable single-joint deviation from the seed."""
        xyz = np.array(xyz_world, float)
        if at_tcp:
            xyz = xyz - TCP * quat_R(quat)[:, 2]
        p_base = xyz - BASE
        if seed is None:
            seed, _ = self.joints()
        best, best_d = None, 1e9
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            pp = req.ik_request.pose_stamped.pose
            pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
            pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            s = np.array(seed, float)
            if k > 0:  # perturb the seed to escape a failed branch
                s = s + np.random.uniform(-0.3, 0.3, size=len(s))
                s = np.clip(s, [l[0] + 0.05 for l in LIMITS], [l[1] - 0.05 for l in LIMITS])
            req.ik_request.robot_state.joint_state.position = [float(v) for v in s]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                q = np.array([sol[j] for j in JOINTS])
                # verify with FK
                got, _ = self.fk(q)
                err = np.linalg.norm(got - xyz)
                if err < 0.005:
                    d = np.abs(q - np.array(seed, float)).max()
                    if d < best_d:
                        best, best_d = q, d
                    if d < 0.6:  # clearly same branch; good enough
                        break
                else:
                    print(f"  ik: FK mismatch {err:.4f}, retry", file=sys.stderr)
            else:
                code = None if res is None else res.error_code.val
                print(f"  ik: fail code={code} (try {k})", file=sys.stderr)
        if best is not None and best_d > max_dist:
            print(f"  ik: best solution is {best_d:.2f} rad from seed (> {max_dist}); rejecting", file=sys.stderr)
            return None
        return best

    # ---------- acting ----------
    def move_joints(self, q_target, duration=None, tol=0.01, max_rounds=6, verbose=True):
        """Send a trajectory; resend until joints are within tol of target."""
        q_target = np.array(q_target, float)
        for r in range(max_rounds):
            q_now, _ = self.joints()
            dq = np.abs(q_target - q_now).max()
            if dq < tol:
                if verbose:
                    print(f"  move_joints: converged (max err {dq:.4f})")
                return True
            dur = duration if (duration and r == 0) else max(1.0, dq / RATE + 0.5)
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(JOINTS)
            pt = JointTrajectoryPoint(positions=[float(v) for v in q_target])
            pt.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=120)
            gh = send.result()
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            q_now, _ = self.joints()
            dq = np.abs(q_target - q_now).max()
            if verbose:
                print(f"  move_joints round {r}: dur={dur:.1f}s code={code} max err {dq:.4f}")
        return dq < tol

    def move_tcp(self, xyz_world, quat, duration=None, seed=None, **kw):
        q = self.ik(xyz_world, quat, seed=seed, at_tcp=True)
        if q is None:
            print(f"  move_tcp: IK failed for {np.round(xyz_world, 3)}")
            return False
        ok = self.move_joints(q, duration, **kw)
        p, _ = self.tcp()
        print(f"  move_tcp: target {np.round(xyz_world, 3)} -> tcp at {np.round(p, 3)} (err {np.linalg.norm(p - xyz_world):.4f})")
        return ok

    def move_tcp_line(self, xyz_from, xyz_to, quat, steps=4, **kw):
        """Straight-ish Cartesian move via several IK waypoints (each seeded
        from the previous) executed one after another."""
        seed, _ = self.joints()
        ok = True
        for i in range(1, steps + 1):
            p = np.array(xyz_from) + (np.array(xyz_to) - np.array(xyz_from)) * i / steps
            q = self.ik(p, quat, seed=seed, at_tcp=True)
            if q is None:
                print(f"  line: IK failed at {np.round(p, 3)}")
                return False
            ok = self.move_joints(q, verbose=False, **kw) and ok
            seed = q
        p, _ = self.tcp()
        print(f"  line: target {np.round(xyz_to, 3)} -> tcp {np.round(p, 3)} (err {np.linalg.norm(p - np.array(xyz_to)):.4f})")
        return ok

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap

    def open(self):
        return self.gripper(GRIP["open_m"])

    def close(self):
        return self.gripper(GRIP["closed_m"])
