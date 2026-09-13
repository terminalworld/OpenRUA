#!/usr/bin/env python3
"""Reusable robot control library for the Panda (one node, clients built once).

Frames: IK/FK operate in panda_link0 (arm base). world->panda_link0 is
translation BASE_T (from TF). Helpers accept world coordinates and
convert.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = TRAJ["joints"]
BASE_T = np.array([0.0, 0.0, 0.0])  # MoveIt model frame IS world here (verified: FK of current pose matches birdview)
TCP = float(M["hand"]["tcp_offset_m"])

# hand pointing straight down, fingers along world Y  (qx,qy,qz,qw)
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_yaw_down(yaw):
    """Hand pointing down (hand +Z = world -Z), rotated about world Z by yaw.
    yaw=0 -> same as Q_DOWN (rotation of pi about X)."""
    # q = qz(yaw) * qx(pi)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    # qx(pi) = (1,0,0,0); qz = (0,0,sy,cy)
    # product (qz * qx): w = cy*0 - sy*0 = 0 ... compute generally
    a = (0.0, 0.0, sy, cy)
    b = (1.0, 0.0, 0.0, 0.0)
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_lib")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, TRAJ["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.fk_cli.wait_for_service(10)
        self.ik_cli.wait_for_service(10)
        self.wait_js()

    def _on_js(self, m):
        self.js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        self.js = None
        t0 = time.time()
        while self.js is None and time.time() - t0 < 10:
            self.spin(0.2)
        return self.js

    def joints(self):
        self.wait_js()
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in ARM]

    def fingers(self):
        self.wait_js()
        d = dict(zip(self.js.name, self.js.position))
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    # ---------- kinematics (base frame) ----------
    def fk(self, q=None, link="panda_hand"):
        q = self.joints() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed: {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))

    def hand_world(self):
        p, q = self.fk()
        return p + BASE_T, q

    def tcp_world(self, q=None):
        p, quat = self.fk(q)
        R = quat_R(*quat)
        return p + BASE_T + TCP * R[:, 2]

    def ik(self, pos_base, quat, seed=None, tries=5):
        seed = self.joints() if seed is None else seed
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.timeout.sec = 1
        req.ik_request.avoid_collisions = False
        for _ in range(tries):
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    def ik_world(self, pos_world, quat, seed=None, at_tcp=True):
        pos = np.array(pos_world, dtype=float) - BASE_T
        if at_tcp:
            R = quat_R(*quat)
            pos = pos - TCP * R[:, 2]
        return self.ik(pos, quat, seed)

    # ---------- motion ----------
    def move_joints(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = list(waypoints or []) + [(q, seconds)]
        for pos, t in wps:
            pt = JointTrajectoryPoint(positions=[float(v) for v in pos])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        h = send.result()
        res = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.joints()) - np.array(q)).max()
        print(f"  move_joints: code={code} max_err={err:.4f}", flush=True)
        return code, err

    def move_tcp_world(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik_world(pos_world, quat, seed)
        if q is None:
            print(f"  IK FAILED for {pos_world}", flush=True)
            return None
        self.move_joints(q, seconds)
        tcp = self.tcp_world()
        print(f"  tcp now {tcp.round(4)} target {np.array(pos_world).round(4)}", flush=True)
        return q

    def servo(self, vx=0.0, vy=0.0, vz=0.0, n=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(dt)

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f


if __name__ == "__main__":
    r = Robot()
    print("joints", np.array(r.joints()).round(4))
    print("fingers", r.fingers())
    p, q = r.hand_world()
    print("hand world", p.round(4), np.array(q).round(4))
    print("tcp world", r.tcp_world().round(4))
    R = quat_R(*q)
    print("hand axes (cols x,y,z):\n", R.round(3))
