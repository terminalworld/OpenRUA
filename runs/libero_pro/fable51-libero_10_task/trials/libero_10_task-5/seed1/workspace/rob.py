#!/usr/bin/env python3
"""Small helper library for this Panda: joint state, FK/IK, trajectory,
gripper, and wrench reads, all in one rclpy node.

On THIS machine the FK/IK model frame is `world` (panda_link0 sits at
world (-0.75, 0, 0.912)); poses here are all world-frame hand poses.
"""
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

BASE = np.array([0.0, 0.0, 0.0])  # FK/IK model frame on this machine is world
M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = M["hand"]["tcp_offset_m"]


def qmul(a, b):
    """Hamilton product of quaternions given as [x, y, z, w]."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


# IK on this machine solves for a tip link rotated -45 deg about hand z
# relative to panda_hand (link8 vs hand); request = hand (x) Rz(+45deg).
QZ45 = np.array([0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Rob:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self._wr.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.traj.wait_for_server(10)
        self.grip.wait_for_server(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---- sensing -------------------------------------------------------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 15:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def fingers(self):
        j = self.joints()
        return j.get("panda_finger_joint1"), j.get("panda_finger_joint2")

    def wrench(self):
        self._wr.pop("m", None)
        t0 = time.time()
        while "m" not in self._wr and time.time() - t0 < 10:
            self.spin(0.2)
        if "m" not in self._wr:
            return None
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z,
                         w.torque.x, w.torque.y, w.torque.z])

    def hand_pose_world(self, q=None):
        """FK: hand pose in WORLD (pos, quat xyzw)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        quat = np.array([p.orientation.x, p.orientation.y,
                         p.orientation.z, p.orientation.w])
        return pos, quat

    # ---- planning ------------------------------------------------------
    def ik_world(self, pos_w, quat, seed=None, at_tcp=False, tries=1):
        """IK for hand (or TCP) pose given in WORLD. Returns joint list or None."""
        pos_w = np.array(pos_w, dtype=float)
        if at_tcp:
            R = quat_to_R(*quat)
            pos_w = pos_w - TCP * R[:, 2]
        pos_b = pos_w - BASE
        quat = qmul(quat, QZ45)  # hand quat -> IK tip-link quat
        if seed is None:
            seed = self.arm_q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos_b)
            (p.orientation.x, p.orientation.y,
             p.orientation.z, p.orientation.w) = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(JOINTS)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
            print(f"  IK fail code={res and res.error_code.val}", file=sys.stderr)
        return None

    # ---- acting --------------------------------------------------------
    def move_q(self, q, seconds=3.0, via=None):
        """Send a trajectory (optionally through `via` points) and wait."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        wps = (via or []) + [q]
        for i, wp in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        r = res.result()
        code = r.result.error_code if r else None
        qn = self.arm_q()
        err = float(np.max(np.abs(np.array(qn) - np.array(q))))
        print(f"  traj code={code} max_joint_err={err:.4f}")
        return code, err

    def move_world(self, pos_w, quat, seconds=3.0, at_tcp=False, seed=None):
        q = self.ik_world(pos_w, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            raise RuntimeError(f"no IK for {pos_w}")
        code, err = self.move_q(q, seconds)
        pos, qt = self.hand_pose_world()
        print(f"  hand now world={pos.round(4)} quat={qt.round(3)}")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f
