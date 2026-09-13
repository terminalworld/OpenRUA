#!/usr/bin/env python3
"""Persistent arm helper: FK/IK/trajectory/gripper clients built once.

TCP = fingertip midpoint, hand +Z (approach) offset by hand.tcp_offset_m.
Hand orientation is "pointing down" with a world yaw psi:
    R = Rz(psi) @ Rx(pi)   ->  hand x = (cos psi, sin psi, 0)
Fingers slide along hand y, so they close along Rz(psi)*(0,-1,0):
    psi = 0     -> fingers close along world y
    psi = pi/2  -> fingers close along world x
"""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def down_R(psi):
    c, s = math.cos(psi), math.sin(psi)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    return Rz @ Rx


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk_cli.wait_for_service(20)
        self.ik_cli.wait_for_service(20)
        self.fjt.wait_for_server(20)
        self.grip.wait_for_server(20)
        self.spin(0.5)

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self._js = {}
        while not self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return [self._js[j] for j in JOINTS]

    def fingers(self):
        self.joints()
        return (self._js.get("panda_finger_joint1"),
                self._js.get("panda_finger_joint2"))

    def _seed(self, q):
        js = JointState()
        js.name = list(JOINTS)
        js.position = [float(v) for v in q]
        return js

    def fk(self, q, link="panda_hand"):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        R = quat_to_R(p.orientation.x, p.orientation.y,
                      p.orientation.z, p.orientation.w)
        return pos, R

    def tcp(self, q=None):
        q = self.joints() if q is None else q
        pos, R = self.fk(q)
        return pos + TCP_OFF * R[:, 2], R

    def ik(self, hand_pos, R, seed=None, timeout=5.0):
        seed = self.joints() if seed is None else seed
        qx, qy, qz, qw = R_to_quat(R)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_pos)
        p.orientation.x, p.orientation.y = float(qx), float(qy)
        p.orientation.z, p.orientation.w = float(qz), float(qw)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout % 1) * 1e9)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=90)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def solve_tcp(self, tcp_xyz, psi, seed=None, pos_tol=0.004):
        """Joint config with TCP at tcp_xyz, hand pointing down, yaw psi.
        IK orientation is loose on this machine -> fix yaw via joint7,
        verify by FK. Returns q or None."""
        R = down_R(psi)
        hand = np.asarray(tcp_xyz, float) - TCP_OFF * R[:, 2]
        cur = self.joints()
        seed = cur if seed is None else seed
        best = None
        for attempt in range(6):
            q = self.ik(hand, R, seed)
            if q is None:
                seed = list(np.array(seed) + np.random.uniform(-0.3, 0.3, 7))
                continue
            for _ in range(3):
                pos, Rf = self.fk(q)
                tilt = math.degrees(math.acos(max(-1, min(1, -Rf[2, 2]))))
                yaw_act = math.atan2(Rf[1, 0], Rf[0, 0])
                dyaw = (psi - yaw_act + math.pi) % (2 * math.pi) - math.pi
                if abs(dyaw) < math.radians(1.0):
                    break
                # joint7 axis = hand z (down) -> +j7 rotates yaw negative
                q[6] -= dyaw
                if not (LIMITS[6][0] < q[6] < LIMITS[6][1]):
                    q[6] += 2 * math.pi if q[6] < 0 else -2 * math.pi
            pos, Rf = self.fk(q)
            tcp_act = pos + TCP_OFF * Rf[:, 2]
            err = np.linalg.norm(tcp_act - tcp_xyz)
            tilt = math.degrees(math.acos(max(-1, min(1, -Rf[2, 2]))))
            yaw_act = math.atan2(Rf[1, 0], Rf[0, 0])
            ok_lim = all(lo < v < hi for v, (lo, hi) in zip(q, LIMITS))
            if err < pos_tol and tilt < 3 and ok_lim and \
                    abs((psi - yaw_act + math.pi) % (2 * math.pi) - math.pi) < math.radians(2):
                dist = max(abs(a - b) for a, b in zip(q, cur))
                if best is None or dist < best[0]:
                    best = (dist, q)
                if dist < 1.0:      # close to where we are: good enough
                    return q
            else:
                print(f"  solve_tcp attempt {attempt}: err={err*1000:.1f}mm tilt={tilt:.1f}deg "
                      f"yaw={math.degrees(yaw_act):.1f} lim_ok={ok_lim}; retrying")
            seed = list(np.array(cur) + np.random.uniform(-0.4, 0.4, 7))
        if best is not None:
            print(f"  solve_tcp: using solution {best[0]:.2f} rad from current")
            return best[1]
        return None

    def move(self, q, seconds, tol=0.02, retries=1):
        # slow down for big joint-space jumps (>= 2 s per rad of travel)
        dist = max(abs(a - b) for a, b in zip(q, self.joints()))
        seconds = max(seconds, 2.0 * dist)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        for attempt in range(retries + 1):
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            gh = send.result()
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            cur = self.joints()
            err = max(abs(a - b) for a, b in zip(cur, q))
            print(f"  move: error_code={code} max_joint_err={err:.4f}")
            if err < tol:
                return True
        return err < tol

    def move_tcp(self, tcp_xyz, psi, seconds=3.0, seed=None):
        q = self.solve_tcp(tcp_xyz, psi, seed)
        if q is None:
            print(f"  NO IK for tcp={tcp_xyz} psi={math.degrees(psi):.0f}")
            return False
        ok = self.move(q, seconds)
        p, R = self.tcp()
        print(f"  tcp now {p.round(4)} yaw={math.degrees(math.atan2(R[1,0],R[0,0])):.1f}")
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
        f = self.fingers()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f
