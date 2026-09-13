#!/usr/bin/env python3
"""Robot helper: FK/IK, trajectory, gripper, joint state -- clients built once."""
import math
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

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE = np.zeros(3)  # FK/IK service already answers in world coords (verified vs TF)
# IK tip link is panda_link8 = panda_hand rotated +45deg about z
RZ45 = np.array([[math.cos(math.pi/4), -math.sin(math.pi/4), 0], [math.sin(math.pi/4), math.cos(math.pi/4), 0], [0, 0, 1]])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    """rotation matrix -> (x,y,z,w)"""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()


def down_quat(yaw=0.0):
    """hand pointing straight down (hand z = -world z), hand x rotated by yaw about world z."""
    from scipy.spatial.transform import Rotation
    R = Rotation.from_euler("xyz", [math.pi, 0, yaw]).as_matrix()
    return R_quat(R)


class Robot:
    def __init__(self, name="rb"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            end = time.time() + 20
            while self._js is None and time.time() < end:
                self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 10
        while self._wr is None and time.time() < end:
            self.spin(0.1)
        if self._wr is None:
            return None
        f = self._wr.wrench.force
        t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    # ---------- kinematics ----------
    def fk(self, q=None, link="panda_hand"):
        """world-frame pose (pos, quat) of link for arm config q (default current)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik(self, pos, quat, seed=None, at_tcp=False, timeout=20.0):
        """IK for a world-frame hand pose. Returns arm q list or None."""
        pos = np.array(pos, dtype=float)
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        pos_b = pos - BASE
        quat = R_quat(quat_R(*quat) @ RZ45)  # hand pose -> link8 pose
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1, nanosec=0)
        if seed is None:
            seed = self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            print("IK: no answer")
            return None
        if res.error_code.val != 1:
            print(f"IK failed code={res.error_code.val}")
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        q = [sol[j] for j in ARM]
        for i, (v, (lo, hi)) in enumerate(zip(q, LIMITS)):
            if v < lo - 1e-3 or v > hi + 1e-3:
                print(f"IK solution violates limit on joint {i+1}: {v}")
                return None
        return q

    # ---------- motion ----------
    def move_q(self, q, seconds=3.0, wait=True):
        return self.move_traj([q], [seconds], wait=wait)

    def move_traj(self, qs, times, wait=True, max_speed=0.3, retries=3):
        """Send trajectory; slow down so no joint exceeds max_speed rad/s;
        on tolerance-violation aborts (-5) resend the remaining goal."""
        q0 = self.arm_q()
        prev, t_prev, scale = q0, 0.0, 1.0
        for q, t in zip(qs, times):
            d = max(abs(a - b) for a, b in zip(q, prev))
            dt = max(t - t_prev, 1e-3)
            scale = max(scale, d / dt / max_speed)
            prev, t_prev = q, t
        if scale > 1.0:
            times = [t * scale for t in times]
            print(f"traj slowed x{scale:.2f} (total {times[-1]:.1f}s)")
        code = self._send_traj(qs, times, wait)
        if not wait:
            return code
        for _ in range(retries):
            q_now = self.arm_q()
            err = max(abs(a - b) for a, b in zip(q_now, qs[-1]))
            if code == 0 or err < 0.02:
                break
            print(f"  retrying final point (err {err:.3f})")
            code = self._send_traj([qs[-1]], [max(3.0, err / max_speed)], True)
        return code

    def _send_traj(self, qs, times, wait=True):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, t in zip(qs, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        if not wait:
            return gh
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = r.result.error_code if r is not None else None
        q_now = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q_now, qs[-1]))
        print(f"traj done code={code} max_joint_err={err:.4f}")
        return code

    def move_pose(self, pos, quat, seconds=3.0, at_tcp=False, seed=None, via=None, max_delta=1.6):
        """IK then move. via: list of intermediate (pos,quat) for one smooth trajectory."""
        seed = seed or self.arm_q()
        qs, ts = [], []
        pts = (via or []) + [(pos, quat)]
        n = len(pts)
        for i, (p, qt) in enumerate(pts):
            q = self.ik(p, qt, seed=seed, at_tcp=at_tcp)
            if q is None:
                print(f"no IK for waypoint {i}: {p}")
                return None
            qs.append(q)
            ts.append(seconds * (i + 1) / n)
            seed = q
        d = [abs(a - b) for a, b in zip(qs[0], self.arm_q())]
        print("first waypoint joint deltas:", np.round(d, 2))
        if max(d) > max_delta:
            print("move_pose: reconfiguration too large, refusing")
            return None
        return self.move_traj(qs, ts)

    def gripper(self, width, wait=True):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        gh = fut.result()
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.fingers()
        print(f"gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f

    def open(self):
        return self.gripper(GRIP["open_m"])

    def close(self):
        return self.gripper(GRIP["closed_m"])

    def report(self):
        pos, quat = self.fk()
        print("hand pos", np.round(pos, 4), "quat", np.round(quat, 4), "fingers", self.fingers())
        return pos, quat
