"""Reusable robot helper: joint state, FK, IK, trajectories, gripper, servo, cameras."""
import math, time, sys
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import TwistStamped, WrenchStamped
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTolerance
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from tf2_msgs.msg import TFMessage

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
BASE = np.array([0.0, 0.0, 0.0])  # FK/IK already answer in world frame (verified)
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2; w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s; y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return np.array([x, y, z, w])


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rotx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def roty(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# Top-down grasp orientation: hand Z points down (-world Z), hand X along world X
# (fingers open along hand Y => along world Y). yaw rotates about world Z.
def R_topdown(yaw=0.0):
    return rotz(yaw) @ np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.wrench = None
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._w_cb, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(20); self.grip.wait_for_server(20)
        self.ik.wait_for_service(20); self.fk.wait_for_service(20)
        self.spin(0.5)

    def _js_cb(self, m):
        self.js = m

    def _w_cb(self, m):
        self.wrench = m

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------- sensing ----------
    def joints(self):
        self.js = None
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in JOINTS])

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def force(self):
        self.wrench = None
        end = time.time() + 2
        while self.wrench is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.wrench is None:
            return None
        f = self.wrench.wrench.force
        return np.array([f.x, f.y, f.z])

    def fk_hand(self, q=None):
        """Hand pose in WORLD frame: (pos, R)."""
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
            raise RuntimeError(f"FK failed {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        R = quat_to_R([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, R

    def tcp(self, q=None):
        pos, R = self.fk_hand(q)
        return pos + TCP * R[:, 2], R

    # ---------- planning ----------
    def ik_hand(self, pos_world, R, seed=None, timeout=20):
        """IK for hand frame at world pos with rotation R. Returns q or None."""
        if seed is None:
            seed = self.arm_q()
        p_base = np.asarray(pos_world) - BASE
        q = R_to_quat(R)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in JOINTS])

    def ik_tcp(self, tcp_world, R, seed=None):
        hand = np.asarray(tcp_world) - TCP * R[:, 2]
        return self.ik_hand(hand, R, seed)

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        if via is not None:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = seconds * (i + 1) / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        # loose path tolerance so lag does not abort the motion mid-way
        goal.path_tolerance = [JointTolerance(name=j, position=2.0) for j in JOINTS]
        goal.goal_tolerance = [JointTolerance(name=j, position=0.01) for j in JOINTS]
        goal.goal_time_tolerance = Duration(sec=5)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = np.abs(self.arm_q() - np.asarray(q)).max()
        print(f"move_q done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_tcp(self, tcp_world, R, seconds=3.0, seed=None):
        q = self.ik_tcp(tcp_world, R, seed)
        if q is None:
            print(f"IK FAILED for tcp {np.round(tcp_world,3)}", flush=True)
            return None
        self.move_q(q, seconds)
        p, _ = self.tcp()
        print(f"  tcp now {np.round(p,4)} (target {np.round(tcp_world,4)})", flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        self.spin(0.3)
        print(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}", flush=True)
        return r

    def servo(self, lin, ang=(0, 0, 0), n=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=dt)

    # ---------- cameras ----------
    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        got = {}
        s1 = self.node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
        s2 = self.node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
        s3 = self.node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
        end = time.time() + 30
        while not all(k in got for k in "cdi") and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        for s in (s1, s2, s3):
            self.node.destroy_subscription(s)
        img = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
        out = out or f"/workspace/snaps/{cam}.png"
        cv2.imwrite(out, img)
        d = got["d"]
        depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
        return img, depth, got["i"]

    def cam_tf(self, cam):
        """world -> <cam>_optical_frame (pos, R) from /tf."""
        got = {}
        frame = f"{cam}_optical_frame"
        qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cb = lambda m: [got.setdefault("t", t) for t in m.transforms if t.child_frame_id == frame]
        s1 = self.node.create_subscription(TFMessage, "/tf", cb, 100)
        s2 = self.node.create_subscription(TFMessage, "/tf_static", cb, qos)
        end = time.time() + 10
        while "t" not in got and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(s1); self.node.destroy_subscription(s2)
        t = got["t"].transform
        return (np.array([t.translation.x, t.translation.y, t.translation.z]),
                quat_to_R([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]))

    def cloud(self, cam):
        """World-frame xyz (H,W,3) for the camera's current depth frame."""
        img, depth, info = self.snap(cam)
        T, R = self.cam_tf(cam)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        v, u = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
        P = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
        return img, P @ R.T + T


def slerp_R(R0, R1, t):
    """Interpolate rotation matrices via axis-angle."""
    from scipy.spatial.transform import Rotation as Rot, Slerp
    s = Slerp([0, 1], Rot.from_matrix(np.stack([R0, R1])))
    return s([t])[0].as_matrix()


def cart_path(r, tcp0, R0, tcp1, R1, n, seed, max_step=0.7):
    """IK along a straight Cartesian TCP path; returns list of q or None."""
    qs = []; q = seed
    for i in range(1, n + 1):
        t = i / n
        p = np.asarray(tcp0) * (1 - t) + np.asarray(tcp1) * t
        R = slerp_R(R0, R1, t)
        qn = None
        for _ in range(4):
            qn = r.ik_tcp(p, R, seed=q)
            if qn is not None and np.abs(qn - q).max() < max_step: break
            qn = None
        if qn is None:
            print(f"cart_path: IK/continuity failed at t={t:.2f}", flush=True); return None
        qs.append(qn); q = qn
    return qs
