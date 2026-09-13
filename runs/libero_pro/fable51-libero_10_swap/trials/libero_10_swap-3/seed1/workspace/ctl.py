#!/usr/bin/env python3
"""Controller helpers for the Panda: FK/IK (MoveIt), trajectories, gripper,
servo bursts, joint-state and camera reads. World<->base conversion built in
(base panda_link0 at world (-0.66, 0, 0.912), identity rotation).
"""
import sys
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK

ARM = [f"panda_joint{i}" for i in range(1, 8)]
# FK/IK on this machine already answer in world coordinates (verified:
# FK hand pose == TF world->panda_hand), so no base offset is applied.
BASE_W = np.array([0.0, 0.0, 0.0])
TCP_OFF = 0.1034


def quat_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_quat(R):
    m = R
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(m))
    if i == 0:
        s = np.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return np.array([0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s])
    if i == 1:
        s = np.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return np.array([(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s])
    s = np.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return np.array([(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s])


def down_quat(yaw):
    """Hand z pointing straight down (-world z); yaw = rotation of the hand
    x-axis about world z. Fingers close along the hand y-axis."""
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])  # x=(c,s,0), y=(s,-c,0), z=(0,0,-1)
    return R_quat(R)


def rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def hand_to_link8(q_hand):
    return R_quat(quat_R(q_hand) @ rotz(np.pi / 4))


class Ctl:
    def __init__(self, name="ctl"):
        rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.wr = None
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._wr_cb, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.wait_js()

    def _js_cb(self, m):
        self.js = m

    def _wr_cb(self, m):
        self.wr = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        self.js = None
        end = time.time() + 15
        while self.js is None and time.time() < end:
            self.spin(0.2)
        return self.joints()

    def joints(self):
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def fingers(self):
        d = self.joints()
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def wrench(self):
        self.wr = None
        end = time.time() + 5
        while self.wr is None and time.time() < end:
            self.spin(0.2)
        f = self.wr.wrench.force
        return np.array([f.x, f.y, f.z])

    # ---------- kinematics ----------
    def fk_world(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, self.arm_q() if q is None else q))
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {None if r is None else r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_W
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q

    def ik_world(self, pos_w, quat, seed=None, at_tcp=False, timeout=20.0):
        pos_w = np.array(pos_w, float)
        if at_tcp:
            pos_w = pos_w - TCP_OFF * quat_R(quat)[:, 2]
        p_b = pos_w - BASE_W
        # machine fact: the IK tip link is panda_link8, which is rotated
        # -45deg about the hand z-axis relative to panda_hand (FK/camera
        # frame). Convert the requested HAND orientation to link8.
        quat = hand_to_link8(quat)
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, p_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(map(float, self.arm_q() if seed is None else seed))
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None:
            raise RuntimeError("IK no answer")
        if r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    # ---------- motion ----------
    def move_q(self, q, seconds=3.0, waypoints=None, retries=3):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        dj7 = abs(self.arm_q()[6] - q[6])
        need = dj7 / 0.15
        if need > seconds and not waypoints:
            print(f"move_q: stretching duration {seconds}->{need:.1f}s for joint7 travel {dj7:.2f}", flush=True)
            seconds = need
        wps = (waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=list(map(float, qq)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        self.wait_js()
        err = np.abs(self.arm_q() - np.array(q)).max()
        print(f"move_q: error_code={code} max_joint_err={err:.4f}", flush=True)
        if err > 0.02 and retries > 0:
            # controller lag (joint 7 is slow on this machine): resend
            return self.move_q(q, max(seconds, 3.0), retries=retries - 1)
        return code, err

    def move_pose(self, pos_w, quat, seconds=3.0, at_tcp=False, seed=None):
        q = self.ik_world(pos_w, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            print(f"IK FAILED for {pos_w} {quat}", flush=True)
            return None
        code, err = self.move_q(q, seconds)
        pos, qq = self.fk_world()
        print(f"  hand now at {pos.round(4)} quat {qq.round(3)}", flush=True)
        return q

    def gripper(self, width, effort=30.0):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(effort)
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res_fut = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res_fut, timeout_sec=120)
        r = res_fut.result().result
        self.wait_js()
        print(f"gripper: reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}", flush=True)
        return r

    def servo(self, v_lin, n=20, frame="panda_link0"):
        msg = TwistStamped()
        msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v_lin)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)
        self.wait_js()

    def snap(self, cam, out=None):
        from cv_bridge import CvBridge
        import cv2
        got = {}
        sub = self.node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("m", m), 1)
        end = time.time() + 20
        while "m" not in got and time.time() < end:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        img = CvBridge().imgmsg_to_cv2(got["m"], "bgr8")
        cv2.imwrite(out or f"{cam}.png", img)
        return img

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()


def move_line(c, p_from, p_to, quat, seconds, n=6, at_tcp=True, seed=None):
    """Straight-ish Cartesian move: IK for n waypoints, one trajectory."""
    p_from, p_to = np.array(p_from, float), np.array(p_to, float)
    wps = []
    q = c.arm_q() if seed is None else np.array(seed)
    for i in range(1, n + 1):
        p = p_from + (p_to - p_from) * i / n
        sol = c.ik_world(p, quat, seed=q, at_tcp=at_tcp)
        if sol is None:
            print(f"move_line: IK failed at waypoint {i} {p}", flush=True)
            return None
        # guard against branch flips
        if np.abs(sol - q).max() > 1.0:
            print(f"move_line: branch jump at waypoint {i}: {np.abs(sol-q).round(2)}", flush=True)
            return None
        q = sol
        wps.append((sol, seconds * i / n))
    last = wps.pop()
    code, err = c.move_q(last[0], last[1], waypoints=wps)
    pos, qq = c.fk_world()
    tcp = pos + TCP_OFF * quat_R(qq)[:, 2]
    print(f"  tcp now {tcp.round(4)}", flush=True)
    return last[0]


def tcp_pose(c):
    pos, qq = c.fk_world()
    return pos + TCP_OFF * quat_R(qq)[:, 2], qq


def rotx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def tilt_quat(yaw, alpha):
    """down_quat(yaw) then tilted about the WORLD x-axis so the hand z-axis
    (finger direction) points down and toward -y by angle alpha; the wrist
    leans toward +y."""
    return R_quat(rotx(-alpha) @ quat_R(down_quat(yaw)))
