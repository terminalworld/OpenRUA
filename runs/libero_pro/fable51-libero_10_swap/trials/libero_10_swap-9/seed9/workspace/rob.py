#!/usr/bin/env python3
"""Small helper library for this Panda workstation (world-frame poses).

IK/FK on this machine take/return WORLD coordinates (verified: FK of the
current state equals TF world->panda_hand; IK with world coords returns the
current joints).
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

M = yaml.safe_load(open('/workspace/machine.yaml'))
ARM = M['actuators'][0]['joints']
LIMITS = M['actuators'][0]['limits_rad']
TCP = M['hand']['tcp_offset_m']


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


def quat_from_axes(hx, hy, hz):
    """Quaternion whose rotation has the given hand axes (world vectors) as columns."""
    R = np.stack([np.asarray(hx, float), np.asarray(hy, float), np.asarray(hz, float)], 1)
    return R_to_quat(R)


class Robot:
    def __init__(self, name='rob'):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, '/joint_states', self._on_js, 1)
        self.node.create_subscription(WrenchStamped, M['sensors'][1]['port'], self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, M['actuators'][0]['port'])
        self.grip = ActionClient(self.node, GripperCommand, M['actuators'][2]['port'])
        self.ik_cli = self.node.create_client(GetPositionIK, '/compute_ik')
        self.fk_cli = self.node.create_client(GetPositionFK, '/compute_fk')
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def _on_js(self, m):
        self._js['m'] = m

    def _on_wr(self, m):
        self._wr['m'] = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.clear()
        while 'm' not in self._js:
            self.spin(0.2)
        d = dict(zip(self._js['m'].name, self._js['m'].position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def fingers(self):
        d = self.joints()
        return d['panda_finger_joint1'], d['panda_finger_joint2']

    def wrench(self):
        self._wr.clear()
        while 'm' not in self._wr:
            self.spin(0.2)
        w = self._wr['m'].wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])

    def fk(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ['panda_hand']
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(self.arm_q() if q is None else q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        p = fut.result().pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def ik(self, pos, quat, seed=None, timeout=5.0, attempts=3):
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = 'panda_arm'
            req.ik_request.pose_stamped.header.frame_id = ''
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = ARM
            req.ik_request.robot_state.joint_state.position = list(map(float, self.arm_q() if seed is None else seed))
            req.ik_request.timeout.sec = int(timeout)
            req.ik_request.timeout.nanosec = int((timeout % 1) * 1e9)
            req.ik_request.avoid_collisions = False
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=120)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return np.array([sol[j] for j in ARM])
        return None

    def move_q(self, q, seconds=3.0, via=None):
        """Send one trajectory (optionally through intermediate points `via`,
        list of (q, t)) and wait; returns error_code."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        for qq, t in (via or []) + [(q, seconds)]:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=120)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(self.arm_q() - np.asarray(q)).max()
        print(f'  move_q: error_code={code} max_joint_err={err:.4f}', flush=True)
        return code

    def move_pose(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed)
        if q is None:
            print(f'  IK FAILED for {np.round(pos, 3)}', flush=True)
            return None
        code = self.move_q(q, seconds)
        p, _ = self.fk()
        print(f'  now at {np.round(p, 4)} (target {np.round(pos, 4)})', flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f'  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}', flush=True)
        return f


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                     w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


# IK on this machine solves for panda_link8; panda_hand = link8 rotated -45deg
# about z. Convert a desired HAND quaternion into the link8 quaternion to ask for.
Q_Z45 = np.array([0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)])


def hand_to_link8(q_hand):
    return quat_mul(np.asarray(q_hand, float), Q_Z45)


_orig_ik = Robot.ik


def _ik_hand(self, pos, quat, **kw):
    return _orig_ik(self, pos, hand_to_link8(quat), **kw)


Robot.ik = _ik_hand


def path_check(r, q0, q1, n=15, verbose=True):
    """FK-sample the straight joint-space path q0->q1; report hand & fingertip
    positions and flag intrusions into the microwave/door/table volumes."""
    bad = 0
    for i in range(n + 1):
        q = q0 + (q1 - q0) * i / n
        p, quat = r.fk(q)
        tip = p + quat_to_R(quat)[:, 2] * TCP
        flags = []
        for name, pt in (('hand', p), ('tip', tip)):
            x, y, z = pt
            if z < 0.93:
                flags.append(f'{name} below table')
            if -0.31 < x < 0.08 and -0.38 < y < -0.12 and z < 1.13:
                flags.append(f'{name} in microwave box')
            if -0.48 < x < -0.28 and -0.62 < y < -0.34 and z < 1.13:
                flags.append(f'{name} in door zone')
        bad += bool(flags)
        if verbose or flags:
            print(f'   {i:2d} hand {np.round(p,3)} tip {np.round(tip,3)} {" ".join(flags)}', flush=True)
    return bad


def move_q_retry(r, q, seconds=3.0, tries=4, tol=0.01):
    """Move to q; resend on lag. If a small static offset remains (payload
    droop), command q minus the residual so the arm lands on q."""
    q = np.asarray(q, float); cmd = q.copy()
    for i in range(tries):
        code = r.move_q(cmd, seconds)
        res = r.arm_q() - q
        err = np.abs(res).max()
        if err < tol:
            return True
        if err < 0.05:
            cmd = cmd - res
        print(f'  retry {i+1}: max_joint_err={err:.3f}', flush=True)
    return False


def Q_yaw(psi_deg):
    """Hand pointing horizontally along d=(sin psi, cos psi, 0), camera (hand x) up."""
    d = np.array([np.sin(np.radians(psi_deg)), np.cos(np.radians(psi_deg)), 0.0])
    hx = np.array([0, 0, 1.0]); hy = np.cross(d, hx)
    return quat_from_axes(hx, hy, d), d


def ik_near(r, pos, quat, seed, tries=6, timeout=1.0, max_dist=None):
    """IK, several attempts, return the solution closest (L-inf) to seed."""
    best = None
    for _ in range(tries):
        s = _orig_ik(r, pos, hand_to_link8(quat), seed=seed, timeout=timeout, attempts=1)
        if s is None:
            continue
        d = np.abs(s - seed).max()
        if best is None or d < best[0]:
            best = (d, s)
        if d < 0.5:
            break
    if best is None:
        return None
    if max_dist is not None and best[0] > max_dist:
        print(f'  ik_near: nearest solution is far (dist {best[0]:.2f})', flush=True)
    return best[1]
