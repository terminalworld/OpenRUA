#!/usr/bin/env python3
"""Reusable robot helpers: joint state, FK, IK, trajectory, gripper, TF, camera."""
import math, struct, sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import WrenchStamped, TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener

ARM = [f"panda_joint{i}" for i in range(1, 8)]
FJT = "/panda_arm_controller/follow_joint_trajectory"
GRIP = "/franka_gripper/gripper_action"
LIMITS = [(-2.9, 2.9), (-1.76, 1.76), (-2.9, 2.9), (-3.07, -0.07), (-2.9, 2.9), (-0.02, 3.75), (-2.9, 2.9)]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_quat(R):
    """rotation matrix -> quaternion (x,y,z,w)"""
    m = R
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s])
    i = np.argmax(np.diag(m))
    if i == 0:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return np.array([0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return np.array([(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s])
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return np.array([(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s])


def frame_quat(z_axis, y_axis):
    """hand orientation from desired hand-z (approach) and hand-y (finger opening) axes in world"""
    z = np.array(z_axis, float); z /= np.linalg.norm(z)
    y = np.array(y_axis, float); y -= y.dot(z) * z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    R = np.column_stack([x, y, z])
    return R_quat(R)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self.node)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT)
        self.grip = ActionClient(self.node, GripperCommand, GRIP)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.1):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                rclpy.spin_once(self.node, timeout_sec=0.2)
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
        end = time.time() + 3
        while self._wr is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self._wr is None:
            return None
        f = self._wr.wrench.force; t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z, t.x, t.y, t.z])

    def fk(self, q=None, link="panda_hand"):
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, q))
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z]), np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik(self, pos, quat, seed=None, timeout=20.0, attempts=1):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"  # default tip is link8 (45deg off)
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(map(float, seed if seed is not None else self.arm_q()))
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=90)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_q(self, q, seconds=3.0, via=None):
        """send trajectory; via = list of (q, t) intermediate points"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if via:
            for vq, vt in via:
                pt = JointTrajectoryPoint(positions=list(map(float, vq)))
                pt.time_from_start = Duration(sec=int(vt), nanosec=int((vt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=list(map(float, q)))
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        qn = self.arm_q()
        err = max(abs(a - b) for a, b in zip(qn, q))
        print(f"  move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_pose(self, pos, quat, seconds=3.0, seed=None, max_jump=1.0):
        q = ik_near(self, pos, quat, seed=seed, max_jump=max_jump)
        if q is None:
            print("  IK FAILED for", pos, quat, flush=True)
            return None
        self.move_q(q, seconds)
        p, _ = self.fk()
        print(f"  hand now at {p.round(4)} (target {np.round(pos,4)})", flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def servo(self, lin, ang=(0, 0, 0), n=20, frame="panda_link0"):
        msg = TwistStamped(); msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def snap(self, cam, out=None):
        """grab color + depth, save png, return (bgr, xyz world HxWx3)"""
        import cv2
        from cv_bridge import CvBridge
        got = {}
        subs = [self.node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1),
                self.node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1),
                self.node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)]
        end = time.time() + 30
        while len(got) < 3 and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        for s in subs:
            self.node.destroy_subscription(s)
        img = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
        d = got["d"]
        D = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
        k = got["i"].k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        frame = f"{cam}_optical_frame"
        end = time.time() + 10
        while time.time() < end and not self.tfbuf.can_transform("world", frame, rclpy.time.Time()):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        t = self.tfbuf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation
        R = quat_R(q.x, q.y, q.z, q.w)
        p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        vs, us = np.mgrid[0:d.height, 0:d.width]
        P = np.stack([(us - cx) * D / fx, (vs - cy) * D / fy, D], -1)
        W = P @ R.T + p0
        if out:
            cv2.imwrite(out, img)
            np.save(out.rsplit(".", 1)[0] + "_xyz.npy", W)
        return img, W


def slerp(q0, q1, t):
    q0 = np.array(q0, float); q1 = np.array(q1, float)
    d = q0.dot(q1)
    if d < 0:
        q1 = -q1; d = -d
    if d > 0.9995:
        q = q0 + t * (q1 - q0); return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - t) * th) * q0 + math.sin(t * th) * q1) / math.sin(th)


def move_cart(r, pos, quat, steps=6, seconds=4.0, max_jump=1.2):
    """IK along a straight line from the current hand pose; one multi-point trajectory.
    Returns final joints or None (no motion) if any IK fails / branch jumps."""
    p0, q0 = r.fk()
    seed = r.arm_q()
    pts = []
    for i in range(1, steps + 1):
        t = i / steps
        p = p0 + (np.array(pos) - p0) * t
        q = slerp(q0, quat, t)
        sol = ik_near(r, p, q, seed=seed, max_jump=max_jump)
        if sol is None:
            print(f"  IK failed at step {i}/{steps} p={p.round(3)}", flush=True)
            return None
        jump = max(abs(a - b) for a, b in zip(sol, seed))
        if jump > max_jump:
            print(f"  branch jump {jump:.2f} at step {i}; abort", flush=True)
            return None
        pts.append((sol, seconds * t))
        seed = sol
    via = pts[:-1]
    code, err = r.move_q(pts[-1][0], seconds, via=via)
    p, qn = r.fk()
    print(f"  hand at {p.round(4)} target {np.round(pos,4)} poserr={np.linalg.norm(p-np.array(pos)):.4f}", flush=True)
    return pts[-1][0]


def hand_from_tcp(tcp, quat):
    """hand-origin position for a desired TCP position and hand orientation"""
    R = quat_R(*quat)
    return np.array(tcp, float) - 0.1034 * R[:, 2]


def tcp_now(r):
    p, q = r.fk()
    return p + 0.1034 * quat_R(*q)[:, 2]


def settle(r, q, tries=3, seconds=2.0, tol=0.01):
    """re-send a joint target until the arm has actually converged"""
    err = max(abs(a - b) for a, b in zip(r.arm_q(), q))
    for _ in range(tries):
        if err < tol:
            break
        _, err = r.move_q(q, seconds)
    return err


def go_tcp(r, tcp, quat, steps=6, seconds=4.0, max_jump=1.2):
    """straight-line move of the TCP, then settle"""
    q = move_cart(r, hand_from_tcp(tcp, quat), quat, steps=steps, seconds=seconds, max_jump=max_jump)
    if q is None:
        return None
    settle(r, q)
    print(f"  TCP now {tcp_now(r).round(4)} target {np.round(tcp,4)}", flush=True)
    return q


def ik_near(r, pos, quat, seed=None, max_jump=1.0, tries=8, timeout=3.0):
    """IK solution close (in joint space) to the seed; perturbs the seed on retries"""
    seed = list(r.arm_q() if seed is None else seed)
    rng = np.random.default_rng(0)
    best = None
    for i in range(tries):
        s = seed if i == 0 else list(np.array(seed) + rng.normal(0, 0.15, 7))
        sol = r.ik(pos, quat, seed=s, timeout=timeout)
        if sol is None:
            continue
        jump = max(abs(a - b) for a, b in zip(sol, seed))
        if best is None or jump < best[0]:
            best = (jump, sol)
        if jump <= max_jump:
            return sol
    print(f"  ik_near: no solution within {max_jump} (best jump {best[0] if best else None})", flush=True)
    return None


LINKS = ["panda_link3", "panda_link4", "panda_link5", "panda_link6", "panda_link7", "panda_hand"]


def path_check(r, q0, q1, n=10, zmin=1.0, boxes=()):
    """FK-sample the joint-space interpolation q0->q1; report min z of links/tcp and box hits.
    boxes: list of (name, xmin,xmax,ymin,ymax,zmin,zmax) forbidden for tcp/hand/link7"""
    worst = 9
    hits = []
    for i in range(n + 1):
        q = [a + (b - a) * i / n for a, b in zip(q0, q1)]
        pts = {}
        for l in LINKS:
            pts[l] = r.fk(q, l)[0]
        p, qq = r.fk(q)
        pts["tcp"] = p + 0.1034 * quat_R(*qq)[:, 2]
        pts["palm"] = p + 0.058 * quat_R(*qq)[:, 2]
        for k, v in pts.items():
            worst = min(worst, v[2])
            for b in boxes:
                if b[1] <= v[0] <= b[2] and b[3] <= v[1] <= b[4] and b[5] <= v[2] <= b[6]:
                    hits.append((i, k, b[0]))
    print(f"  path_check: min z {worst:.3f} hits {hits[:6]}", flush=True)
    return worst, hits


CABINET = ("cabinet", -0.14, 0.15, 0.03, 0.42, 0.85, 1.14)
SHELF = ("shelf", -0.35, -0.05, -0.62, -0.16, 0.85, 1.20)
