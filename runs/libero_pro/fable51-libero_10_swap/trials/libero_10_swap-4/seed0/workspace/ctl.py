#!/usr/bin/env python3
"""Reusable controller: IK -> trajectory, gripper, joint/FK readers.
World<->base: base = world + (0.51, 0, -0.42) (from TF world->panda_link0)."""
import sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
import cv2

ARM = [f"panda_joint{i}" for i in range(1, 8)]
# Verified: /compute_fk and /compute_ik poses are already in WORLD coords
# (FK of the current state matched TF world->panda_hand), so no offset.
BASE_OFF = np.zeros(3)
TCP = 0.1034
# IK tip link is panda_link8 (= panda_hand rotated +45deg about Z), so
# these are link8 orientations. Both are top-down (hand Z = world -Z):
# fingers opening along world X
Q_FX = (0.9238795, 0.3826834, 0.0, 0.0)
# fingers opening along world Y
Q_FY = (0.9238795, -0.3826834, 0.0, 0.0)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Ctl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("ctl")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._js_cb, 10)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand,
                                 "/franka_gripper/gripper_action")
        self.twist_pub = self.node.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10)
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fk.wait_for_service(10), "no FK"
        assert self.fjt.wait_for_server(10), "no FJT"
        assert self.grip.wait_for_server(10), "no gripper"
        self.wait_js()

    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        self.js = {}
        end = time.time() + 20
        while not self.js and time.time() < end:
            self.spin(0.2)
        return dict(self.js)

    def arm_q(self):
        js = self.wait_js()
        return [js[j] for j in ARM]

    def fingers(self):
        js = self.wait_js()
        return js["panda_finger_joint1"], js["panda_finger_joint2"]

    def _call(self, cli, req, timeout=60):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    def fk_hand(self, q=None):
        """Hand pose in WORLD frame (position, quat xyzw)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(q if q is not None else self.arm_q())
        res = self._call(self.fk, req)
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_OFF
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp_world(self, q=None):
        pos, quat = self.fk_hand(q)
        R = quat_to_R(*quat)
        return pos + TCP * R[:, 2], quat

    def solve_ik(self, pos_world, quat, at_tcp=True, seed=None):
        pos = np.array(pos_world, float)
        R = quat_to_R(*quat)
        if at_tcp:
            pos = pos - TCP * R[:, 2]
        pos = pos - BASE_OFF
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(seed if seed is not None else self.arm_q())
        req.ik_request.timeout = Duration(sec=2)
        req.ik_request.avoid_collisions = False
        res = self._call(self.ik, req)
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def traj(self, points, secs):
        """points: list of 7-vectors; secs: list of time_from_start."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        for q, t in zip(points, secs):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        # controller may report before the arm has converged: poll until
        # the joint state stops changing, then compare with the target
        prev = np.array(self.arm_q())
        for _ in range(40):
            q = np.array(self.arm_q())
            if np.abs(q - prev).max() < 1e-4:
                break
            prev = q
        err = np.abs(q - np.array(points[-1])).max()
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        if err > 0.02:
            print("  resending goal to converge")
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            q = np.array(self.arm_q())
            err = np.abs(q - np.array(points[-1])).max()
            print(f"  retry code={res.result().result.error_code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos, quat, secs=3.0, seed=None, via=None):
        q = self.solve_ik(pos, quat, seed=seed)
        if q is None:
            print(f"  IK FAILED for {np.round(pos,3)}")
            return None
        pts, ts = [], []
        if via is not None:
            pts.append(via); ts.append(secs * 0.5)
        pts.append(q); ts.append(secs)
        self.traj(pts, ts)
        tcp, _ = self.tcp_world()
        print(f"  tcp now {np.round(tcp,3)} (target {np.round(pos,3)})")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        f = self.fingers()
        print(f"  gripper -> fingers {f[0]:.4f} {f[1]:.4f}")
        return f

    def servo(self, v, n, dt=0.05):
        """Stream a base-frame linear velocity v (m/s) for n ticks."""
        msg = TwistStamped()
        msg.header.frame_id = "panda_link0"
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(dt)

    def snap(self, cam, out=None):
        got = {}
        topic = f"/{cam}/color/image_raw"
        sub = self.node.create_subscription(Image, topic, lambda m: got.setdefault("m", m), 1)
        end = time.time() + 30
        while "m" not in got and time.time() < end:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        img = CvBridge().imgmsg_to_cv2(got["m"], "bgr8")
        out = out or f"{cam}.png"
        cv2.imwrite(out, img)
        return img

    def depth_cloud(self, cam):
        """Return (H,W,3) world points + color image for a camera (needs TF)."""
        from tf2_ros import Buffer, TransformListener
        buf = Buffer(); TransformListener(buf, self.node)
        got = {}
        s1 = self.node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
        s2 = self.node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("k", m), 1)
        s3 = self.node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
        end = time.time() + 30
        while len(got) < 3 and time.time() < end:
            self.spin(0.2)
        for s in (s1, s2, s3):
            self.node.destroy_subscription(s)
        frame = f"{cam}_optical_frame"
        end = time.time() + 10
        while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
            self.spin(0.2)
        t = buf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        d = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float32)
        K = np.array(got["k"].k).reshape(3, 3)
        h, w = d.shape
        vs, us = np.mgrid[0:h, 0:w]
        p = np.stack([(us - K[0, 2]) * d / K[0, 0], (vs - K[1, 2]) * d / K[1, 1], d], -1)
        W = p.reshape(-1, 3) @ R.T + tr
        img = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
        return W.reshape(h, w, 3), img
