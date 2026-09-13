#!/usr/bin/env python3
"""Small control library for this Panda: joint read, FK/IK (MoveIt), multi-point
trajectories, gripper.  Poses are given in WORLD frame and converted to the arm
base (planner model frame) internally.  Import and use from scripts."""
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
ARM = FJT["joints"]
LIMITS = FJT["limits_rad"]
# Verified via /compute_fk: the planner's model frame IS `world` on this machine
# (panda_link0 sits at (-0.66, 0, 0.912) in it), so no offset is applied.
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])
TCP = M["hand"]["tcp_offset_m"]
IK_LINK = "panda_hand"   # group tip is panda_link8, rotated 45 deg about z from the hand


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def R_to_quat(R):
    """Rotation matrix -> (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_from_axes(x_hand=None, y_hand=None, z_hand=None):
    """Build hand rotation from two of its axes given in world coords."""
    if z_hand is not None and y_hand is not None:
        z = np.asarray(z_hand, float); z /= np.linalg.norm(z)
        y = np.asarray(y_hand, float); y -= z * (y @ z); y /= np.linalg.norm(y)
        x = np.cross(y, z)
    elif z_hand is not None and x_hand is not None:
        z = np.asarray(z_hand, float); z /= np.linalg.norm(z)
        x = np.asarray(x_hand, float); x -= z * (x @ z); x /= np.linalg.norm(x)
        y = np.cross(z, x)
    else:
        raise ValueError
    return np.stack([x, y, z], axis=1)


def R_down(yaw=0.0):
    """Hand pointing straight down; fingers open along world y rotated by yaw."""
    z = np.array([0, 0, -1.0])
    y = np.array([-math.sin(yaw), -math.cos(yaw), 0.0])  # yaw=0: fingers along -y (panda default-ish)
    return R_from_axes(z_hand=z, y_hand=y)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])

    def _on_js(self, msg):
        self._js = msg

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        """dict name->position (all joints)."""
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self, fresh=True):
        j = self.joints(fresh)
        return np.array([j[n] for n in ARM])

    def finger_gap(self, fresh=True):
        j = self.joints(fresh)
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand", timeout=60):
        """World pose (p, quat) of link for arm joints q (default: current)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        if not self.fk_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no FK service")
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        q_ = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q_

    def tcp(self, q=None):
        """World position of the TCP (fingertip centre)."""
        p, quat = self.fk(q)
        R = quat_to_R(quat)
        return p + TCP * R[:, 2], R

    # ---------------- planning ----------------
    def ik(self, p_world, R, seed=None, at_tcp=True, timeout=60, attempts=4, max_dev=0.8):
        """Joint solution for hand pose. p_world is the TCP (fingertip) point if
        at_tcp else the hand frame origin. Returns np.array or None."""
        p = np.asarray(p_world, float)
        if at_tcp:
            p = p - TCP * R[:, 2]
        pb = p - BASE_IN_WORLD
        quat = R_to_quat(R)
        if seed is None:
            seed = self.arm_q()
        if not self.ik_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no IK service")
        best = None
        for k in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.ik_link_name = IK_LINK
            req.ik_request.pose_stamped.header.frame_id = ""
            ps = req.ik_request.pose_stamped.pose
            ps.position.x, ps.position.y, ps.position.z = map(float, pb)
            ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            s = np.asarray(seed, float)
            if k > 0:  # perturb seed on retry (wider each time)
                s = s + np.random.uniform(-0.3 * k, 0.3 * k, size=7)
                s = np.clip(s, [l[0] + 0.05 for l in LIMITS], [l[1] - 0.05 for l in LIMITS])
            req.ik_request.robot_state.joint_state.position = [float(v) for v in s]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=2)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
            res = fut.result()
            if res is None:
                log("IK: no answer")
                continue
            if res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                qs = np.array([sol[n] for n in ARM])
                dev = np.abs(qs - np.asarray(seed, float)).max()
                if best is None or dev < best[0]:
                    best = (dev, qs)
                if dev < max_dev:  # close enough to the seed: take it
                    return qs
                log(f"IK attempt {k}: solution deviates {dev:.2f} rad from seed")
                continue
            log(f"IK attempt {k}: error {res.error_code.val}")
        if best is not None:
            log(f"IK: returning best solution (dev {best[0]:.2f})")
            return best[1]
        return None

    # ---------------- acting ----------------
    def _send_traj(self, waypoints, durations, timeout=900):
        if not self.traj.wait_for_server(timeout_sec=10):
            raise RuntimeError("no trajectory server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(waypoints, durations):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        if rf.result() is None:
            log("trajectory: no result within timeout")
            return None
        return rf.result().result.error_code

    def settle(self, target, tol=0.01, max_iter=20):
        """Poll joints until they stop changing; return (err, q)."""
        prev = self.arm_q()
        for _ in range(max_iter):
            q = self.arm_q()
            if np.abs(q - prev).max() < 1e-4:
                break
            prev = q
        err = np.abs(q - np.asarray(target)).max()
        return err, q

    def move_joints(self, waypoints, durations=None, speed=0.5, min_time=2.0,
                    tol=0.01, retries=3):
        """Move through joint waypoints (durations cumulative seconds; if None,
        derived from `speed` rad/s per segment). Verifies arrival from
        /joint_states and resends the final point if off by > tol."""
        waypoints = [np.asarray(w, float) for w in waypoints]
        if durations is None:
            q = self.arm_q()
            durations, t = [], 0.0
            for w in waypoints:
                t += max(min_time, np.abs(w - q).max() / speed)
                durations.append(round(t, 2)); q = w
        code = self._send_traj(waypoints, durations)
        err, q = self.settle(waypoints[-1])
        log(f"trajectory code={code} err={err:.4f} (dur {durations[-1]}s)")
        n = 0
        while err > tol and n < retries:
            n += 1
            d = max(min_time, err / speed)
            log(f"  re-sending final point (err {err:.4f}, {d:.1f}s)")
            code = self._send_traj([waypoints[-1]], [d])
            err, q = self.settle(waypoints[-1])
            dq = np.asarray(q) - waypoints[-1]
            log(f"  code={code} err={err:.4f} worst j{int(np.argmax(np.abs(dq)))+1} dq={np.round(dq, 3)}")
        return err

    def move_to_pose(self, p_world, R, seed=None, at_tcp=True, speed=0.5,
                     retries=3, tol=0.01, **kw):
        q = self.ik(p_world, R, seed=seed, at_tcp=at_tcp, **kw)
        if q is None:
            raise RuntimeError(f"IK failed for {p_world}")
        err = self.move_joints([q], speed=speed, retries=retries, tol=tol)
        t, _ = self.tcp()
        log(f"  tcp now {np.round(t, 4)} (target {np.round(p_world, 4)}) joint err {err:.4f}")
        return q, err

    def move_line(self, p_to, R, step=0.03, speed=0.15, retries=6, tol=0.006, R_to=None):
        """Straight-line TCP move from the current pose to p_to (optionally
        interpolating hand yaw about world z to R_to), IK per waypoint seeded
        by the previous, sent as one trajectory; then settle on the last point."""
        p0, _ = self.tcp()
        p_to = np.asarray(p_to, float)
        n = max(1, int(np.ceil(np.linalg.norm(p_to - p0) / step)))
        if R_to is not None:
            a0 = math.atan2(R[1, 1], R[0, 1]); a1 = math.atan2(R_to[1, 1], R_to[0, 1])
            da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
            n = max(n, int(np.ceil(abs(da) / 0.15)))
        seed = self.arm_q()
        wps, durs, t = [], [], 0.0
        for i in range(1, n + 1):
            f = i / n
            p = p0 + f * (p_to - p0)
            Ri = R
            if R_to is not None:
                c, s_ = math.cos(f * da), math.sin(f * da)
                Rz = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
                Ri = Rz @ R
            q = self.ik(p, Ri, seed=seed, max_dev=0.5)
            if q is None:
                q = self.ik(p, Ri, seed=seed, max_dev=0.5, attempts=8)
            if q is None:
                raise RuntimeError(f"IK failed on line at {p}")
            t += max(0.6, np.abs(q - seed).max() / speed)
            wps.append(q); durs.append(round(t, 2)); seed = q
        return self.move_joints(wps, durs, speed=speed, retries=retries, tol=tol)

    def gripper(self, width, timeout=300):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        gap = self.finger_gap()
        log(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap


if __name__ == "__main__":
    r = Robot()
    q = r.arm_q()
    print("arm q:", np.round(q, 4))
    print("finger gap:", r.finger_gap())
    p, quat = r.fk(q)
    print("hand world pos:", np.round(p, 4), "quat:", np.round(quat, 4))
    t, R = r.tcp(q)
    print("tcp world:", np.round(t, 4))
    print("hand axes (cols x,y,z):\n", np.round(R, 3))
