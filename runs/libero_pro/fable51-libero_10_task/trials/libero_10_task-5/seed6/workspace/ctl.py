#!/usr/bin/env python3
"""Reusable robot controller: joint state, FK/IK (MoveIt), trajectories,
gripper, camera snapshots. Poses are WORLD frame; converted to the arm
base frame (machine.yaml planning_frame) for MoveIt."""
import sys, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_IN_WORLD = None  # filled from TF


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_quat(R):
    # returns x,y,z,w
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


def topdown_quat(yaw=0.0):
    """Hand z pointing down (world -z); yaw rotates finger axis about world z.
    yaw=0: fingers open along world y. yaw=pi/2: along world x."""
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    R = Rz @ np.diag([1.0, -1.0, -1.0])
    return R_quat(R)


class Robot:
    def __init__(self, name="ctl"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 1)
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self.node)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.bridge = CvBridge()
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        while not self.tfbuf.can_transform("world", "panda_link0", Time()):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        # Verified on this machine: /compute_fk and /compute_ik with empty
        # frame_id use WORLD coordinates (FK of panda_hand == TF world->hand),
        # so no base offset is applied.
        self.base = np.zeros(3)

    def _js(self, m):
        self.js = m

    def spin(self, n=5, dt=0.05):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=dt)

    def joints(self):
        self.js = None
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints(); return [d[j] for j in ARM]

    def finger(self):
        d = self.joints(); return d["panda_finger_joint1"], d["panda_finger_joint2"]

    # ---------- kinematics ----------
    def hand_pose(self):
        """world pose of panda_hand from FK service (avoids stale TF)."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        q = self.arm_q()
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = q
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + self.base
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik_solve(self, pos_world, quat, seed=None, collide=False):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"  # default tip is panda_link8 (yawed -45deg)
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pb = np.array(pos_world) - self.base
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = seed if seed is not None else self.arm_q()
        f1, f2 = self.finger()  # bridge reports finger2 negative; MoveIt wants both >= 0
        req.ik_request.robot_state.joint_state.name = list(ARM) + ["panda_finger_joint1", "panda_finger_joint2"]
        req.ik_request.robot_state.joint_state.position = list(map(float, seed)) + [abs(f1), abs(f2)]
        req.ik_request.robot_state.is_diff = True  # keep scene-attached objects (held mug)
        req.ik_request.avoid_collisions = bool(collide)
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    # ---------- motion ----------
    def move_joints(self, positions, seconds=3.0, retries=2):
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(ARM)
            pt = JointTrajectoryPoint(positions=list(map(float, positions)))
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            q = np.array(self.arm_q()); err = np.abs(q - np.array(positions)).max()
            print(f"  move: code={code} max_joint_err={err:.4f}", flush=True)
            if err < 0.02:
                return True
        return False

    def move_pose(self, pos_world, quat, seconds=3.0, seed=None, max_delta=1.0, collide=True):
        q = self.ik_solve(pos_world, quat, seed, collide=collide)
        if q is None:
            print("  IK failed for", pos_world, flush=True); return False
        d = np.abs(np.array(q) - np.array(self.arm_q())).max()
        if d > max_delta:
            print(f"  REFUSED: IK solution is {d:.2f} rad away (max_delta={max_delta})", flush=True); return False
        ok = self.move_joints(q, seconds)
        p, _ = self.hand_pose()
        print(f"  hand now at {p.round(4)} (target {np.array(pos_world).round(4)})", flush=True)
        return ok

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        f = self.finger()
        print(f"  gripper: reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}", flush=True)
        return f

    def servo(self, lin, n=20, ang=(0, 0, 0)):
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg); rclpy.spin_once(self.node, timeout_sec=0.05)

    # ---------- vision ----------
    def snap(self, cam, out=None):
        got = {}
        s1 = self.node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
        s2 = self.node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
        s3 = self.node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
        while len(got) < 3:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        for s in (s1, s2, s3):
            self.node.destroy_subscription(s)
        color = self.bridge.imgmsg_to_cv2(got["c"], "bgr8")
        depth = self.bridge.imgmsg_to_cv2(got["d"], "passthrough").astype(float)
        k = got["i"].k
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, color)
        frame = f"{cam}_optical_frame"
        while not self.tfbuf.can_transform("world", frame, Time()):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        t = self.tfbuf.lookup_transform("world", frame, Time())
        q = t.transform.rotation
        T = np.eye(4); T[:3, :3] = quat_R(q.x, q.y, q.z, q.w)
        T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        h, w = depth.shape; u, v = np.meshgrid(np.arange(w), np.arange(h))
        pc = np.stack([(u - k[2]) * depth / k[0], (v - k[5]) * depth / k[4], depth, np.ones_like(depth)], -1)
        P = (T @ pc.reshape(-1, 4).T).T[:, :3].reshape(h, w, 3)
        return color, depth, P, T
