#!/usr/bin/env python3
"""Arm helper: persistent clients for IK, trajectory, gripper, joint state.

World frame -> planning frame (panda_link0) offset comes from TF at start.
Usage as a library, or: python3 -u arm.py <script.py>  (exec's the file with
`A` bound to an Arm instance).
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
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
DOWN = (1.0, 0.0, 0.0, 0.0)  # hand z down, fingers open along world y


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_down(yaw):
    """Quaternion: hand pointing down, fingers opening along world direction
    rotated by `yaw` from world y (yaw=0 -> DOWN)."""
    # DOWN = rot_x(pi). Compose rot_z(yaw) * rot_x(pi)
    cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    # q_z = (0,0,sz,cz), q_x = (1,0,0,0); product q_z*q_x:
    # (w1w2 - v1.v2, w1 v2 + w2 v1 + v1 x v2)
    w = cz * 0 - (0 * 1 + 0 * 0 + sz * 0)
    v = cz * np.array([1, 0, 0]) + 0 * np.array([0, 0, sz]) + np.cross([0, 0, sz], [1, 0, 0])
    return (v[0], v[1], v[2], w)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.ik.wait_for_service(timeout_sec=20)
        self.fjt.wait_for_server(timeout_sec=20)
        self.grip.wait_for_server(timeout_sec=20)
        self.wait_js()
        # world -> planning frame offset
        t0 = time.time()
        while time.time() - t0 < 10 and not self.tfbuf.can_transform(
                "world", M["planning"]["planning_frame"], rclpy.time.Time()):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        t = self.tfbuf.lookup_transform("world", M["planning"]["planning_frame"],
                                        rclpy.time.Time())
        self.base_off = np.array([t.transform.translation.x,
                                  t.transform.translation.y,
                                  t.transform.translation.z])
        self.log(f"TF world->planning frame: {self.base_off}")
        # empirically (FK/IK round-trip) MoveIt's model frame here IS world:
        # poses with empty frame_id are taken in world coordinates
        self.base_off = np.zeros(3)
        self.ik_link = "panda_hand"

    def log(self, *a):
        print(time.strftime("%H:%M:%S"), *a, flush=True)

    def _on_js(self, msg):
        self.js = dict(zip(msg.name, msg.position))

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        t0 = time.time()
        while not self.js and time.time() - t0 < 20:
            self.spin(0.2)
        return self.js

    def fresh_js(self):
        self.js = {}
        return self.wait_js()

    def arm_q(self):
        js = self.fresh_js()
        return [js[j] for j in JOINTS]

    def fingers(self):
        js = self.fresh_js()
        return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    def hand_pose_world(self):
        """world -> panda_hand via TF (chain world->link0->...->hand)."""
        for _ in range(20):
            self.spin(0.1)
        try:
            t = self.tfbuf.lookup_transform("world", M["frames"]["hand"], rclpy.time.Time())
        except Exception as e:  # noqa
            self.log("TF hand lookup failed:", e)
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), (q.x, q.y, q.z, q.w)

    def tcp_world(self):
        hp = self.hand_pose_world()
        if hp is None:
            return None
        p, q = hp
        R = quat_to_R(*q)
        return p + TCP * R[:, 2]

    # ---- IK -----------------------------------------------------------
    def solve_ik(self, pos_world, quat=DOWN, at_tcp=True, seed=None, timeout=30.0):
        pos = np.array(pos_world, dtype=float)
        R = quat_to_R(*quat)
        if at_tcp:
            pos = pos - TCP * R[:, 2]
        pos_pf = pos - self.base_off
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = self.ik_link
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_pf)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        s = JointState()
        s.name = list(JOINTS)
        s.position = [float(v) for v in (seed if seed is not None else self.arm_q())]
        req.ik_request.robot_state.joint_state = s
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            self.log("IK: no answer")
            return None
        if res.error_code.val != 1:
            self.log(f"IK failed code={res.error_code.val} for world {pos_world} (pf {pos_pf})")
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        q = [sol[j] for j in JOINTS]
        return q

    def solve_ik_retry(self, pos_world, quat=DOWN, at_tcp=True, tries=6):
        cur = self.arm_q()
        q = self.solve_ik(pos_world, quat, at_tcp, seed=cur)
        if q is not None:
            return q
        seeds = [
            [0.0, -0.4, 0.0, -2.2, 0.0, 1.9, 0.785],
            [0.0, 0.0, 0.0, -1.8, 0.0, 1.9, 0.785],
            [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            [0.0, 0.3, 0.0, -1.5, 0.0, 1.9, 0.785],
        ]
        for i, s in enumerate(seeds[:tries]):
            q = self.solve_ik(pos_world, quat, at_tcp, seed=s)
            if q is not None:
                return q
        return None

    # ---- motion -------------------------------------------------------
    def move_joints(self, q, seconds=3.0, tol=0.02, resend=2):
        for attempt in range(resend + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(JOINTS)
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            gh = send.result()
            if gh is None or not gh.accepted:
                self.log("trajectory goal not accepted")
                continue
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            r = res.result()
            code = r.result.error_code if r else None
            cur = self.arm_q()
            err = max(abs(a - b) for a, b in zip(cur, q))
            self.log(f"traj done code={code} max_joint_err={err:.4f}")
            if err < tol:
                return True
        return err < tol

    def move_to(self, pos_world, quat=DOWN, seconds=3.0, at_tcp=True):
        q = self.solve_ik_retry(pos_world, quat, at_tcp)
        if q is None:
            self.log(f"NO IK for {pos_world}")
            return False
        ok = self.move_joints(q, seconds)
        tcp = self.tcp_world()
        self.log(f"move_to {np.round(pos_world,3)} -> ok={ok} tcp_now={None if tcp is None else np.round(tcp,3)}")
        return ok

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result()
        f = self.fingers()
        self.log(f"gripper({width}) reached={r.result.reached_goal if r else None} "
                 f"stalled={r.result.stalled if r else None} fingers={f}")
        return f

    def servo(self, vx=0.0, vy=0.0, vz=0.0, ticks=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(dt)
        self.log(f"servo burst done tcp={np.round(self.tcp_world(),3)}")

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    A = Arm()
    try:
        if len(sys.argv) > 1:
            exec(open(sys.argv[1]).read(), {"A": A, "np": np, "DOWN": DOWN, "yaw_down": yaw_down})
        else:
            A.log("arm q:", np.round(A.arm_q(), 3))
            A.log("fingers:", A.fingers())
            A.log("hand world:", A.hand_pose_world())
            A.log("tcp world:", A.tcp_world())
    finally:
        A.shutdown()
