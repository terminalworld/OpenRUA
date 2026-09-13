#!/usr/bin/env python3
"""Small control library for this Panda: IK, FK, trajectories, gripper, sensing.

All poses are WORLD frame; converted to the arm base (panda_link0) for MoveIt.
"""
import sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import WrenchStamped, TwistStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK

# Verified empirically: /compute_fk and /compute_ik on this machine speak
# WORLD coordinates (FK of the start pose matched the birdview camera), so
# no base offset is applied.
BASE = np.array([0.0, 0.0, 0.0])
ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)          # hand z down, fingers along world y
Q_DOWN_X = (0.7071068, 0.7071068, 0.0, 0.0)  # hand z down, fingers along world x


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Ctl:
    def __init__(self, name="ctl"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def js(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin()
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.js()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.js()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        while self._wr is None:
            self.spin()
        f = self._wr.wrench.force; t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def fk(self, q=None):
        """world-frame position + quaternion of panda_hand."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = [float(v) for v in (q or self.arm_q())]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = quat_R(*quat)
        return pos + TCP * R[:, 2], quat

    # ---------- planning ----------
    def ik(self, pos_world, quat, at_tcp=True, seed=None, tries=3):
        pos = np.array(pos_world, dtype=float)
        if at_tcp:
            pos = pos - TCP * quat_R(*quat)[:, 2]
        pos_b = pos - BASE
        seed = seed or self.arm_q()
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = "panda_arm"
            req.ik_request.ik_link_name = "panda_hand"   # group tip is link8 (45deg off)
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos_b)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = ARM
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.timeout.sec = 2
            req.ik_request.avoid_collisions = False
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[n] for n in ARM]
            print(f"IK try {k}: error {r and r.error_code.val}", file=sys.stderr)
        return None

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, via=None):
        # machine quirk: joint7 tracks at only ~0.17 rad/s; budget time for it
        d7 = abs(q[6] - self.arm_q()[6])
        seconds = max(seconds, d7 / 0.15)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if via:
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
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"move_q: code={code} max_err={err:.4f}")
        return code, err

    def move_pose(self, pos_world, quat, seconds=3.0, at_tcp=True, seed=None):
        q = self.ik(pos_world, quat, at_tcp=at_tcp, seed=seed)
        if q is None:
            print("move_pose: IK FAILED, no motion")
            return None
        self.move_q(q, seconds)
        return q

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        f = self.fingers()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def servo(self, lin, ang=(0, 0, 0), ticks=20, frame="panda_link0"):
        msg = TwistStamped(); msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            self.twist_pub.publish(msg); self.spin(0.05)

    def snap(self, cam, out=None):
        from cv_bridge import CvBridge
        import cv2
        got = []
        sub = self.node.create_subscription(Image, f"/{cam}/color/image_raw", got.append, 1)
        while not got:
            self.spin(0.5)
        self.node.destroy_subscription(sub)
        out = out or f"{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got[0], "bgr8"))
        return out
