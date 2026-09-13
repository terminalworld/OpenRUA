#!/usr/bin/env python3
"""Reusable arm helpers: joint state, FK, IK, trajectory, gripper, servo.

Kept in one long-lived node so clients are built once.
"""
import math
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP_OFF = 0.1034
LIMITS = [(-2.9, 2.9), (-1.76, 1.76), (-2.9, 2.9), (-3.07, -0.07),
          (-2.9, 2.9), (-0.02, 3.75), (-2.9, 2.9)]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# hand pointing straight down, fingers closing along world Y
# (hand X = world X, hand Y = world -Y, hand Z = world -Z): 180 deg about X
Q_DOWN_X = (1.0, 0.0, 0.0, 0.0)
# hand pointing down, fingers closing along world X: rotate Q_DOWN_X by 90deg about Z
Q_DOWN_Y = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)


def q_down_yaw(yaw):
    """Hand pointing down with the hand X axis rotated by yaw about world Z."""
    # q = qz(yaw) * (1,0,0,0)
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    # qz = (0,0,s,c); qx180 = (1,0,0,0); product qz*qx180:
    # w = c*0 - 0*1 - 0*0 - s*0 = 0 ; x = c*1 + 0 + 0*0 - s*0 = c
    # y = c*0 - 0*0 + 0*1 + s*0 ... careful: use generic multiply
    return quat_mul((0, 0, s, c), (1, 0, 0, 0))


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


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
        self.traj = ActionClient(self.node, FollowJointTrajectory,
                                 "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand,
                                 "/franka_gripper/gripper_action")
        self.twist_pub = self.node.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.fk_cli.wait_for_service(10)
        self.ik_cli.wait_for_service(10)
        self.traj.wait_for_server(10)
        self.grip.wait_for_server(10)

    def _on_js(self, m):
        self._js["m"] = m
        self._js["t"] = time.time()

    def spin(self, sec=0.2):
        rclpy.spin_once(self.node, timeout_sec=sec)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        d = dict(zip(self._js["m"].name, self._js["m"].position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, q))
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        quat = (p.orientation.x, p.orientation.y, p.orientation.z,
                p.orientation.w)
        return pos, quat

    def tcp(self, q=None):
        pos, quat = self.fk(q)
        R = quat_R(*quat)
        return pos + TCP_OFF * R[:, 2], quat

    def ik(self, pos, quat, seed=None, tcp=True, attempts=3):
        """pos: target position (world frame). If tcp, pos is the fingertip
        centre; converted to hand frame origin."""
        pos = np.array(pos, float)
        if tcp:
            R = quat_R(*quat)
            pos = pos - TCP_OFF * R[:, 2]
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        # the IK tip link is panda_link8 = panda_hand rotated +45deg about z
        q8 = quat_mul(quat, (0.0, 0.0, math.sin(math.pi / 8),
                             math.cos(math.pi / 8)))
        (p.orientation.x, p.orientation.y, p.orientation.z,
         p.orientation.w) = map(float, q8)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(map(float, seed))
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        for _ in range(attempts):
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        code = None if res is None else res.error_code.val
        raise RuntimeError(f"IK failed code={code} for pos={pos} quat={quat}")

    def move_q(self, q, seconds=3.0, via=None):
        """Send one trajectory to joint config q (optionally through via
        points, each (q, t))."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if via:
            for vq, vt in via:
                pt = JointTrajectoryPoint(positions=list(map(float, vq)))
                pt.time_from_start = Duration(sec=int(vt),
                                              nanosec=int((vt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=list(map(float, q)))
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        handle = send.result()
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        r = res.result()
        code = r.result.error_code if r else None
        now = self.arm_q()
        err = max(abs(a - b) for a, b in zip(now, q))
        print(f"  traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed)
        code, err = self.move_q(q, seconds)
        tp, _ = self.tcp()
        print(f"  tcp now {tp.round(4)} target {np.array(pos).round(4)}")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def servo(self, vx=0, vy=0, vz=0, ticks=20):
        msg = TwistStamped()
        msg.header.frame_id = "panda_link0"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)
