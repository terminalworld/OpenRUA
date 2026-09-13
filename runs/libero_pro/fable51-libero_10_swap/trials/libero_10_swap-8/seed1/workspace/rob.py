#!/usr/bin/env python3
"""Small helper library for this Panda: joint state, FK/IK, trajectories,
gripper, servo bursts, camera grabs. One rclpy node per process."""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import CameraInfo, Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
# Verified empirically: /compute_fk and /compute_ik poses are already in
# the same frame as px2world's "world" (FK of the current config matches
# TF world->panda_hand), so no base offset is applied.
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])
TCP = M["hand"]["tcp_offset_m"]


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
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def down_quat(yaw):
    """Hand Z pointing down (-world Z), hand X rotated by yaw about world Z.
    yaw=0 -> hand X along +world X (fingers close along world Y)."""
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    Rflip = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # 180 deg about X
    return R_to_quat(Rz @ Rflip)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.bridge = CvBridge()

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---- sensing ----
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def fk_world(self, q=None, link="panda_hand"):
        """Return (pos_world, quat xyzw) of link for arm config q."""
        if q is None:
            q = self.arm_q()
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def tcp_world(self, q=None):
        pos, quat = self.fk_world(q)
        R = quat_to_R(*quat)
        return pos + TCP * R[:, 2], quat

    def grab(self, topic, msg_type=Image, timeout=30.0):
        got = {}
        sub = self.node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
        t0 = time.time()
        while "m" not in got and time.time() - t0 < timeout:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        if "m" not in got:
            raise RuntimeError(f"no msg on {topic}")
        return got["m"]

    def color(self, cam):
        return self.bridge.imgmsg_to_cv2(self.grab(f"/{cam}/color/image_raw"), "bgr8")

    def depth(self, cam):
        return self.bridge.imgmsg_to_cv2(self.grab(f"/{cam}/depth/image_raw"), "passthrough")

    def K(self, cam):
        return np.array(self.grab(f"/{cam}/color/camera_info", CameraInfo).k).reshape(3, 3)

    # ---- acting ----
    def ik_world(self, pos_world, quat, seed=None, tcp=False, tries=1):
        """IK for the hand (or TCP if tcp=True) at a world pose; returns q or None."""
        pos = np.array(pos_world, float)
        if tcp:
            pos = pos - TCP * quat_to_R(*quat)[:, 2]
        pos = pos - BASE_IN_WORLD
        self.ik.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"  # group tip is link8 (45 deg off)
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in (seed if seed is not None else self.arm_q())]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        for _ in range(tries):
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    def move_q(self, q_or_list, seconds, wait=True):
        """Send one or several waypoints (list of q) as one trajectory."""
        pts = q_or_list if isinstance(q_or_list[0], (list, tuple, np.ndarray)) else [q_or_list]
        # this controller tracks at most ~0.17 rad/s per joint (measured);
        # stretch the duration so the goal is reachable in time
        q0 = np.array(self.arm_q())
        dmax = 0.0
        for q in pts:
            dmax += float(np.max(np.abs(np.array(q) - q0)))
            q0 = np.array(q)
        seconds = max(seconds, dmax / 0.12)
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        n = len(pts)
        for i, q in enumerate(pts):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            t = seconds * (i + 1) / n
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q_now = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q_now, pts[-1]))
        print(f"[move] error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        self.grip.wait_for_server(10)
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"[grip] reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap

    def servo(self, v, ticks, w=(0, 0, 0)):
        """Stream a base-frame twist (m/s) for `ticks` messages."""
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, w)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)


def pitched_quat(p, yaw=0.0):
    """Hand approach axis tilted by p rad from straight-down toward the
    +X direction (rotated by yaw about world Z); fingers close along the
    horizontal axis perpendicular to the approach. p=0 == down_quat(yaw)."""
    Z = np.array([math.sin(p), 0.0, -math.cos(p)])
    Y = np.array([0.0, -1.0, 0.0])
    X = np.cross(Y, Z)
    R = np.column_stack([X, Y, Z])
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    return R_to_quat(Rz @ R)


def nice_seed(pos_world):
    """Elbow-up ready pose facing the target azimuth (base at x=-0.66)."""
    az = math.atan2(pos_world[1], pos_world[0] + 0.66)
    return [az, 0.2, 0.0, -2.0, 0.0, 2.2, 0.785 + az]


def ik_nice(r, pos_world, quat, tcp=True, tries=6):
    """IK from several seeds around the nominal ready pose; returns the
    solution closest to the nominal seed (avoids wrapped-around configs)."""
    seed0 = nice_seed(pos_world)
    best = None
    rng = np.random.default_rng(0)
    for i in range(tries):
        seed = seed0 if i == 0 else list(np.array(seed0) + rng.normal(0, 0.3, 7))
        sol = r.ik_world(pos_world, quat, seed=seed, tcp=tcp, tries=1)
        if sol is None:
            continue
        d = float(np.linalg.norm(np.array(sol) - np.array(seed0)))
        if best is None or d < best[0]:
            best = (d, sol)
    return None if best is None else best[1]


def line_move(r, p_from, p_to, quat, n, seconds, seed=None):
    """Straight Cartesian TCP line as one multi-waypoint trajectory."""
    pts = []
    q_prev = seed or r.arm_q()
    for i in range(1, n + 1):
        p = np.array(p_from) + (np.array(p_to) - np.array(p_from)) * i / n
        q = r.ik_world(p, quat, seed=q_prev, tcp=True, tries=3)
        if q is None:
            raise RuntimeError(f"IK failed at waypoint {i}: {p}")
        if np.max(np.abs(np.array(q) - np.array(q_prev))) > 1.0:
            raise RuntimeError(f"IK branch jump at waypoint {i}: {np.round(q,2)} vs {np.round(q_prev,2)}")
        pts.append(q)
        q_prev = q
    return r.move_q(pts, seconds)


LIMITS = np.array(FJT["limits_rad"])


def margin_ok(q, m=0.12):
    q = np.array(q)
    return bool(np.all(q > LIMITS[:, 0] + m) and np.all(q < LIMITS[:, 1] - m))


def pitched_quat2(p, flip=False, yaw=0.0):
    """Like pitched_quat; flip=True rotates the hand 180 deg about its
    approach axis (same grasp, fingers swapped, other IK branches)."""
    Z = np.array([math.sin(p), 0.0, -math.cos(p)])
    Y = np.array([0.0, 1.0 if flip else -1.0, 0.0])
    X = np.cross(Y, Z)
    R = np.column_stack([X, Y, Z])
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    return R_to_quat(Rz @ R)


def ik_best(r, pos_world, quat, seed0=None, tries=8, tcp=True):
    """Several seeds; keep solutions with joint-limit margin; return the one
    closest to seed0 (default: nice_seed)."""
    seed0 = np.array(seed0 if seed0 is not None else nice_seed(pos_world))
    rng = np.random.default_rng(1)
    best = None
    for i in range(tries):
        seed = seed0 if i == 0 else seed0 + rng.normal(0, 0.4, 7)
        seed = np.clip(seed, LIMITS[:, 0] + 0.1, LIMITS[:, 1] - 0.1)
        sol = r.ik_world(pos_world, quat, seed=list(seed), tcp=tcp, tries=1)
        if sol is None or not margin_ok(sol):
            continue
        d = float(np.linalg.norm(np.array(sol) - seed0))
        if best is None or d < best[0]:
            best = (d, sol)
    return None if best is None else best[1]


LINKS = ["panda_link1", "panda_link2", "panda_link3", "panda_link4", "panda_link5",
         "panda_link6", "panda_link7", "panda_link8", "panda_hand"]


def links_world(r, q):
    """Positions of all arm link origins in world for config q -> dict."""
    r.fk.wait_for_service(10)
    req = GetPositionFK.Request()
    req.fk_link_names = list(LINKS)
    req.robot_state.joint_state.name = list(ARM)
    req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    res = fut.result()
    out = {}
    for name, ps in zip(res.fk_link_names, res.pose_stamped):
        p = ps.pose.position
        out[name] = np.array([p.x, p.y, p.z]) + BASE_IN_WORLD
    return out


def arm_clearance(r, q, verbose=False):
    """Min z of link origins 3..hand and the hand-frame sanity; crude table check."""
    L = links_world(r, q)
    zs = {k: round(v[2], 3) for k, v in L.items()}
    if verbose:
        for k in LINKS:
            print(f"  {k:12s} {np.round(L[k],3)}")
    return min(L[k][2] for k in LINKS[2:])


def path_ik(r, p_from, p_to, q_from, q_to, n, seed, m=0.1, maxjump=0.6):
    """IK along a straight TCP line with quaternion slerp-ish (linear yaw), seeded consecutively.
    Returns list of q or raises."""
    from scipy.spatial.transform import Rotation as Rot, Slerp
    key = Rot.from_quat([q_from, q_to])
    sl = Slerp([0, 1], key)
    pts = []
    prev = np.array(seed, float)
    for i in range(1, n + 1):
        t = i / n
        pos = np.array(p_from) * (1 - t) + np.array(p_to) * t
        qt = sl([t])[0].as_quat()
        s = r.ik_world(pos, qt, seed=prev, tcp=True, tries=3)
        if s is None:
            raise RuntimeError(f"IK fail at t={t:.2f} pos={np.round(pos,3)}")
        s = np.array(s)
        jump = np.abs(s - prev).max()
        if jump > maxjump:
            raise RuntimeError(f"joint jump {jump:.2f} at t={t:.2f} q={np.round(s,2)} prev={np.round(prev,2)}")
        if not margin_ok(s, m):
            raise RuntimeError(f"margin violated at t={t:.2f} q={np.round(s,2)}")
        pts.append(s)
        prev = s
    return pts


def plan_chain(r, wps, n_per=4, m=0.1, tries=25, seed_ref=None, rng=None, maxjump=0.6):
    """wps: list of (pos, quat). Try many IK starts at wps[0]; follow with path_ik.
    Returns list of segments (each a list of q) or None."""
    rng = rng or np.random.default_rng(0)
    pos0, qt0 = wps[0]
    starts = []
    for i in range(tries):
        ref = seed_ref if seed_ref is not None else nice_seed(pos0)
        s = r.ik_world(np.array(pos0), qt0, seed=ref + rng.normal(0, 0.7, 7), tcp=True)
        if s is not None and margin_ok(np.array(s), m):
            starts.append(np.array(s))
    best = None
    for s in starts:
        segs = []
        prev = s
        try:
            for (pa, qa), (pb, qb) in zip(wps[:-1], wps[1:]):
                seg = path_ik(r, pa, pb, qa, qb, n_per, prev, m=m, maxjump=maxjump)
                segs.append(seg)
                prev = seg[-1]
        except RuntimeError as e:
            continue
        allq = np.array([q for seg in segs for q in seg])
        mg = float(np.min(np.minimum(allq - LIMITS[:, 0], LIMITS[:, 1] - allq)))
        if best is None or mg > best[0]:
            best = (mg, s, segs)
    return best


def wrench(r):
    """External wrench (fx,fy,fz,tx,ty,tz) from the robot state broadcaster."""
    from geometry_msgs.msg import WrenchStamped
    m = r.grab("/franka_robot_state_broadcaster/external_wrench", WrenchStamped)
    f, t = m.wrench.force, m.wrench.torque
    return np.array([f.x, f.y, f.z, t.x, t.y, t.z])
