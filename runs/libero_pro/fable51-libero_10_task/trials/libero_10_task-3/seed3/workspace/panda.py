#!/usr/bin/env python3
"""Robot helper library: FK/IK (numpy), trajectory, gripper, sensors.

World frame = panda_link0 + BASE_OFF.  Poses handled as 4x4 matrices.
TCP = panda_hand origin + 0.1034 along hand z (fingertip pad centre).
"""
import math
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_OFF = np.array([-0.66, 0.0, 0.912])
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
LIM = np.array([[-2.9, 2.9], [-1.76, 1.76], [-2.9, 2.9], [-3.07, -0.07],
                [-2.9, 2.9], [-0.02, 3.75], [-2.9, 2.9]])
TCP_OFF = 0.1034
# modified DH: (a, d, alpha)
DH = [(0, 0.333, 0), (0, 0, -math.pi / 2), (0, 0.316, math.pi / 2),
      (0.0825, 0, math.pi / 2), (-0.0825, 0.384, -math.pi / 2),
      (0, 0, math.pi / 2), (0.088, 0, math.pi / 2)]


def _tf(a, d, alpha, theta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([[ct, -st, 0, a],
                     [st * ca, ct * ca, -sa, -d * sa],
                     [st * sa, ct * sa, ca, d * ca],
                     [0, 0, 0, 1]])


def rotz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def fk_hand_base(q):
    """panda_hand pose in panda_link0 frame."""
    T = np.eye(4)
    for (a, d, al), th in zip(DH, q):
        T = T @ _tf(a, d, al, th)
    T = T @ _tf(0, 0.107, 0, 0) @ rotz(-math.pi / 4)
    return T


def fk_tcp(q):
    """TCP pose in WORLD frame."""
    T = fk_hand_base(q)
    T = T @ np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, TCP_OFF], [0, 0, 0, 1]])
    T[:3, 3] += BASE_OFF
    return T


def pose_err(T, Tt):
    """6-vector: position error and rotation error (axis-angle) of T vs target."""
    dp = Tt[:3, 3] - T[:3, 3]
    Rd = Tt[:3, :3] @ T[:3, :3].T
    ang = math.acos(max(-1.0, min(1.0, (np.trace(Rd) - 1) / 2)))
    if ang < 1e-9:
        w = np.zeros(3)
    else:
        w = ang / (2 * math.sin(ang)) * np.array([Rd[2, 1] - Rd[1, 2], Rd[0, 2] - Rd[2, 0], Rd[1, 0] - Rd[0, 1]])
    return np.concatenate([dp, w])


def ik(Tt, q0, iters=200, tol=1e-4, pos_only=False, w_rot=1.0):
    """Damped least squares IK for TCP target Tt (world). Returns q or None."""
    q = np.array(q0, float).copy()
    for it in range(iters):
        T = fk_tcp(q)
        e = pose_err(T, Tt)
        if pos_only:
            e[3:] = 0
        e[3:] *= w_rot
        if np.linalg.norm(e[:3]) < tol and np.linalg.norm(e[3:]) < tol * 10:
            return q
        # numeric jacobian
        J = np.zeros((6, 7))
        h = 1e-6
        for i in range(7):
            dq = np.zeros(7); dq[i] = h
            Ti = fk_tcp(q + dq)
            J[:, i] = -pose_err(Ti, Tt) + e  # (e(q) - e(q+dq))/h * ... sign handled below
        J /= h
        # J now approximates d(-e)/dq = d(pose)/dq ; solve J dq = e
        lam = 1e-3
        dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), e)
        # limit step
        n = np.linalg.norm(dq)
        if n > 0.3:
            dq *= 0.3 / n
        q = q + dq
        q = np.clip(q, LIM[:, 0] + 0.01, LIM[:, 1] - 0.01)
    e = pose_err(fk_tcp(q), Tt)
    if np.linalg.norm(e[:3]) < 1e-3 and (pos_only or np.linalg.norm(e[3:]) < 1e-2):
        return q
    return None


def make_T(p, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def R_from_axes(z, x=None, y=None):
    """Rotation with given hand z (approach) and hand x or y direction."""
    z = np.array(z, float); z /= np.linalg.norm(z)
    if x is not None:
        x = np.array(x, float); x -= x.dot(z) * z; x /= np.linalg.norm(x)
        y = np.cross(z, x)
    else:
        y = np.array(y, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
        x = np.cross(y, z)
    return np.stack([x, y, z], 1)


def topdown(yaw=0.0):
    """Hand z down; hand y (finger axis) at `yaw` from world y... precisely:
    hand x = (cos yaw, sin yaw, 0), z = -Z."""
    x = np.array([math.cos(yaw), math.sin(yaw), 0])
    return R_from_axes([0, 0, -1], x=x)


class Robot:
    def __init__(self, name="panda_lib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip_cli = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.fjt.wait_for_server(timeout_sec=20)
        self.grip_cli.wait_for_server(timeout_sec=20)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            t0 = time.time()
            while self._js is None and time.time() - t0 < 20:
                self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def q(self):
        d = self.joints()
        return np.array([d[j] for j in JOINTS])

    def fingers(self):
        d = self.joints()
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def wrench(self):
        self._wr = None
        t0 = time.time()
        while self._wr is None and time.time() - t0 < 10:
            self.spin(0.1)
        if self._wr is None:
            return None
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def tcp(self):
        return fk_tcp(self.q())

    def move_traj(self, qs, times, timeout=600):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for qv, t in zip(qs, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in qv])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("goal rejected", flush=True)
            return None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        if rf.result() is None:
            print("traj result timeout", flush=True)
            return None
        code = rf.result().result.error_code
        return code

    def move_q(self, qt, T=None, vmax=0.15):
        qc = self.q()
        Tmin = float(np.abs(np.array(qt) - qc).max()) / vmax + 0.5
        T = Tmin if T is None else max(T, Tmin)
        code = self.move_traj([qt], [T])
        qa = self.q()
        err = np.abs(qa - np.array(qt)).max()
        print(f"move_q code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        fut = self.grip_cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result if rf.result() else None
        f = self.fingers()
        print(f"gripper({width}) reached={getattr(r,'reached_goal',None)} stalled={getattr(r,'stalled',None)} fingers={f}", flush=True)
        return f

    def move_tcp(self, Tt, T=None, q0=None, n_wp=1, vmax=0.15):
        """IK then trajectory to target TCP pose (world). n_wp>1 -> straight-line waypoints."""
        q0 = self.q() if q0 is None else np.array(q0)
        if n_wp <= 1:
            qt = ik(Tt, q0)
            if qt is None:
                print("IK failed", flush=True)
                return None
            return self.move_q(qt, T)
        T0 = fk_tcp(q0)
        qs, ts = [], []
        qprev = q0
        for i in range(1, n_wp + 1):
            s = i / n_wp
            Ti = interp_T(T0, Tt, s)
            qi = ik(Ti, qprev)
            if qi is None:
                print(f"IK failed at waypoint {i}", flush=True)
                return None
            qs.append(qi); qprev = qi
        # time parametrise by joint velocity cap
        ts, t = [], 0.0
        qprev = q0
        for qi in qs:
            t += max(float(np.abs(qi - qprev).max()) / vmax, 0.05)
            ts.append(t); qprev = qi
        if T is not None and T > ts[-1]:
            ts = [x * T / ts[-1] for x in ts]
        ts = [x + 0.3 for x in ts]
        code = self.move_traj(qs, ts)
        qa = self.q()
        err = np.abs(qa - qs[-1]).max()
        print(f"move_tcp code={code} max_joint_err={err:.4f}", flush=True)
        return code, err


def slerp_R(R0, R1, s):
    Rd = R1 @ R0.T
    ang = math.acos(max(-1.0, min(1.0, (np.trace(Rd) - 1) / 2)))
    if ang < 1e-9:
        return R0
    w = np.array([Rd[2, 1] - Rd[1, 2], Rd[0, 2] - Rd[2, 0], Rd[1, 0] - Rd[0, 1]]) / (2 * math.sin(ang))
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    a = ang * s
    Rs = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
    return Rs @ R0


def interp_T(T0, T1, s):
    T = np.eye(4)
    T[:3, :3] = slerp_R(T0[:3, :3], T1[:3, :3], s)
    T[:3, 3] = T0[:3, 3] * (1 - s) + T1[:3, 3] * s
    return T


if __name__ == "__main__":
    r = Robot()
    q = r.q()
    print("q", q.round(4))
    T = fk_tcp(q)
    np.set_printoptions(precision=4, suppress=True)
    print("TCP world:\n", T)
    print("fingers", r.fingers())
    print("wrench", r.wrench())
