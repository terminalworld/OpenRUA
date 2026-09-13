#!/usr/bin/env python3
"""Small helper library for this Panda: FK/IK, trajectory, gripper, snaps."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

import cv2

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
# FK/IK on this machine already answer in WORLD coords (verified: FK of
# the home pose gives the hand at world (-0.20, 0, 1.27) and IK of that
# same world pose succeeds while the base-frame version fails -31).
BASE = np.zeros(3)
_c = math.sqrt(0.5)
RZ45 = np.array([[_c, -_c, 0.0], [_c, _c, 0.0], [0.0, 0.0, 1.0]])
TCP = M["hand"]["tcp_offset_m"]


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
        s = math.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def R_from_axes(zaxis, xaxis):
    """Rotation whose columns are hand x, y, z axes (world), given desired
    z (approach) and approximate x (finger-opening axis)."""
    z = np.asarray(zaxis, float); z /= np.linalg.norm(z)
    x = np.asarray(xaxis, float); x = x - z * (x @ z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.bridge = CvBridge()

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        """Hand pose in WORLD frame: (pos[3], quat[4], R[3,3])."""
        if q is None:
            q = self.arm_q()
        self.fk_cli.wait_for_service(5)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q, quat_to_R(q)

    def tcp(self, q=None):
        pos, quat, R = self.fk(q)
        return pos + TCP * R[:, 2], R

    def ik(self, pos_world, R, seed=None, at_tcp=True, timeout=20.0):
        """IK for hand pose (world). Returns joint array or None."""
        pos = np.asarray(pos_world, float)
        if at_tcp:
            pos = pos - TCP * R[:, 2]
        pos = pos - BASE
        # /compute_ik solves for the group's tip link panda_link8, which is
        # rotated 45 deg about z relative to panda_hand (verified via FK:
        # R_link8 = R_hand @ Rz(+45deg)); same origin, same z axis.
        qt = R_to_quat(np.asarray(R) @ RZ45)
        self.ik_cli.wait_for_service(5)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, qt)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        if seed is None:
            seed = self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout + 30)
        res = fut.result()
        if res is None:
            print("IK: no answer"); return None
        if res.error_code.val != 1:
            print(f"IK failed code={res.error_code.val}"); return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    # ---------------- acting ----------------
    def move_q(self, q_list, seconds_list):
        """Send one trajectory through the given joint waypoints."""
        if not self.fjt.wait_for_server(10):
            raise RuntimeError("no fjt server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, s in zip(q_list, seconds_list):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(s), nanosec=int((s % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        qn = self.arm_q()
        err = np.abs(qn - np.asarray(q_list[-1])).max()
        print(f"traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_to(self, q, seconds=3.0):
        return self.move_q([q], [seconds])

    def gripper(self, width):
        if not self.grip.wait_for_server(10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={gap}")
        return gap

    def servo(self, lin, ang=(0, 0, 0), n=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(dt)

    def snap(self, cam, out=None):
        topic = f"/{cam}/color/image_raw"
        got = []
        sub = self.node.create_subscription(Image, topic, got.append, 1)
        while not got:
            self.spin(0.5)
        self.node.destroy_subscription(sub)
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, self.bridge.imgmsg_to_cv2(got[0], "bgr8"))
        return out


LO = np.array([l[0] for l in FJT["limits_rad"]])
HI = np.array([l[1] for l in FJT["limits_rad"]])
HOME = np.array([0, -0.161, 0, -2.445, 0, 2.227, 0.785])


def tilt_R(tilt_deg):
    """Hand pointing down, tilted back toward -y by tilt_deg (fingertips
    at +y of the wrist). Fingers open along hand y; here hand y = world x
    (hand x = world +y-ish), so the pinch axis is world x and the 0.2 m
    wide hand body lies along x, its 0.06 m thickness along y."""
    t = np.radians(tilt_deg)
    return R_from_axes([0, np.sin(t), -np.cos(t)], [0, np.cos(t), np.sin(t)])


def best_ik(r, pos, R, seeds=None, at_tcp=True, timeout=3.0, near=None):
    """Try several seeds; return the solution with best limit margin
    (or nearest to `near` if given)."""
    if seeds is None:
        seeds = [r.arm_q(), HOME]
    found = []
    for s in seeds:
        sol = r.ik(pos, R, seed=s, at_tcp=at_tcp, timeout=timeout)
        if sol is not None:
            margin = np.minimum(sol - LO, HI - sol).min()
            found.append((margin, sol))
    if not found:
        return None
    if near is not None:
        return min(found, key=lambda f: np.abs(f[1] - near).max())[1]
    return max(found, key=lambda f: f[0])[1]


def goto_q(r, q, seconds=4.0, tries=4, tol=0.01):
    """Resend until the joints settle within tol of q. A first -5 is usually
    controller lag: resend unchanged. A repeatable small residual is
    compensated by offsetting the command; a growing residual means contact
    -> stop pushing."""
    q = np.asarray(q, float)
    cmd = q.copy()
    prev = None
    for i in range(tries):
        code, _ = r.move_q([cmd], [seconds])
        qn = r.arm_q()
        err = np.abs(qn - q).max()
        print(f"  settle err vs target={err:.4f}")
        if err < tol:
            return True
        if prev is not None and err > prev * 1.2:
            print("  residual growing -> probable contact, stop")
            return False
        if prev is not None and abs(err - prev) < 0.3 * prev and err < 0.06:
            cmd = np.clip(cmd + (q - qn), LO, HI)
        prev = err
        seconds = max(1.5, seconds / 2)
    return False


def goto(r, pos, R, seconds=4.0, seeds=None, near=None, at_tcp=True):
    q = best_ik(r, pos, R, seeds=seeds, at_tcp=at_tcp, near=near)
    if q is None:
        print("goto: IK failed for", pos)
        return None
    ok = goto_q(r, q, seconds)
    tcp, _ = r.tcp()
    print(f"goto {np.round(pos,3)} -> tcp {np.round(tcp,3)} ok={ok}")
    return q


def wrench(r, n=1):
    from geometry_msgs.msg import WrenchStamped
    got = []
    sub = r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", got.append, 1)
    while len(got) < n:
        r.spin(0.5)
    r.node.destroy_subscription(sub)
    w = got[-1].wrench
    return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])
