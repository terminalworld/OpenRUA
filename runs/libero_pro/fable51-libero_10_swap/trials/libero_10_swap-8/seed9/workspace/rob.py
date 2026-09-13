"""Reusable helpers for this Panda: joint state, FK/IK, trajectory, gripper,
servo, camera snapshots.  World <-> base offset from TF (world->panda_link0).
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])   # from /tf_static world->panda_link0
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
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.bridge = CvBridge()
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.wait_js()

    def _on_js(self, msg):
        self.js = dict(zip(msg.name, msg.position))

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        self.js = {}
        while not self.js:
            self.spin(0.2)
        return dict(self.js)

    def arm_q(self):
        js = self.wait_js()
        return np.array([js[j] for j in JOINTS])

    def fingers(self):
        js = self.wait_js()
        return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    # ---- kinematics (base frame) ----
    def _seed(self, q):
        s = JointState()
        s.name = list(JOINTS)
        s.position = [float(v) for v in q]
        return s

    def fk_base(self, q=None, link="panda_hand"):
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp_world(self, q=None):
        # NOTE: measured fact: /compute_fk and /compute_ik on this machine work
        # in the WORLD frame (empty frame_id == "world"), not panda_link0.
        p, quat = self.fk_base(q)
        R = quat_to_R(*quat)
        return p + R[:, 2] * TCP, quat

    def ik_base(self, pos, quat, seed=None, timeout=1.5):
        """pos: hand-frame position in WORLD frame (model frame). Returns joint array or None."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(self.arm_q() if seed is None else seed)
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in JOINTS])

    def ik_tcp_world(self, tcp_world, quat, seed=None, timeout=1.5):
        """IK for a TCP position given in WORLD frame with desired panda_hand
        quaternion quat.  Measured fact: /compute_ik solves for a tip frame
        rotated -45 deg about z from panda_hand (panda_link8), so the request
        orientation is R_hand @ Rz(+45deg).  Position is shared."""
        R = quat_to_R(*quat)
        hand_world = np.asarray(tcp_world) - R[:, 2] * TCP
        c, s_ = np.cos(np.pi / 4), np.sin(np.pi / 4)
        Rz45 = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
        q_req = R_to_quat(R @ Rz45)
        return self.ik_base(hand_world, q_req, seed, timeout)

    # ---- motion ----
    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        if via is not None:
            for i, v in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                t = seconds * (i + 1) / (len(via) + 1)
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(self.arm_q() - np.asarray(q)).max()
        print(f"  move_q: error_code={code} max joint err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.fingers()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def servo(self, lin, ang=(0, 0, 0), ticks=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            self.spin(dt)

    # ---- vision ----
    def snap(self, cam, out=None):
        topic = cam if cam.startswith("/") else f"/{cam}/color/image_raw"
        out = out or f"/workspace/{cam.strip('/').split('/')[0]}.png"
        got = []
        sub = self.node.create_subscription(Image, topic, got.append, 1)
        while not got:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        msg = got[0]
        if "FC" in msg.encoding or "16UC" in msg.encoding:
            d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            np.save(out.rsplit(".", 1)[0] + ".npy", d)
            return d
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(out, img)
        return img

    def depth(self, cam):
        return self.snap(f"/{cam}/depth/image_raw", f"/workspace/{cam}_depth.png")


def tf_lookup(node, child, parent="world", secs=3.0):
    """Read the latest transform parent->child straight off /tf (no buffer staleness)."""
    from tf2_msgs.msg import TFMessage
    got = {}
    def cb(m):
        for t in m.transforms:
            if t.header.frame_id == parent and t.child_frame_id == child:
                got["t"] = t.transform
    sub = node.create_subscription(TFMessage, "/tf", cb, 50)
    end = time.time() + secs
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if "t" in got and time.time() > end - secs + 1.0:
            break
    node.destroy_subscription(sub)
    if "t" not in got:
        return None
    tr = got["t"]
    R = quat_to_R(tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)
    p = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
    return p, R


def cloud_from_depth(depth, K, p, R):
    """Depth image (m) + intrinsics + camera pose -> Nx3 world points."""
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    v, u = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
    pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1).reshape(-1, 3)
    return pc @ R.T + p
