#!/usr/bin/env python3
"""Small control library: IK (world frame, verified), trajectory, gripper, FK.

Usage as CLI:
  python3 arm.py tcp X Y Z [secs]          # move TCP (fingertip centre) to X Y Z, hand pointing down, fingers along world y
  python3 arm.py tcp X Y Z secs yawdeg     # same, with yaw about world z (0 = fingers along y)
  python3 arm.py grip open|close
  python3 arm.py state                     # print joints, hand pose, tcp pose, finger gap
"""
import math
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP_OFF = 0.1034


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def down_quat(yaw_deg=0.0):
    """Hand z pointing to world -z; yaw about world z. yaw=0 -> fingers along world y."""
    # base: 180 deg about x -> (1,0,0,0); then yaw about world z: q = qz * qx
    h = math.radians(yaw_deg) / 2
    qz = (0.0, 0.0, math.sin(h), math.cos(h))
    qx = (1.0, 0.0, 0.0, 0.0)
    # quaternion product qz * qx
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_lib")
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 1)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory,
                                 "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.ik_cli.wait_for_service(timeout_sec=20)
        self.fk_cli.wait_for_service(timeout_sec=20)
        self.traj.wait_for_server(timeout_sec=20)
        self.grip.wait_for_server(timeout_sec=20)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, n=5, dt=0.1):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=dt)

    def joints(self):
        self._js = None
        while self._js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        t0 = time.time()
        while self._wr is None and time.time() - t0 < 3:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self._wr is None:
            return None
        f = self._wr.wrench.force
        return (f.x, f.y, f.z)

    def fk(self, q=None):
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = JointState(name=ARM, position=list(q))
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p, o = r.pose_stamped[0].pose.position, r.pose_stamped[0].pose.orientation
        return np.array([p.x, p.y, p.z]), (o.x, o.y, o.z, o.w)

    def tcp(self):
        p, q = self.fk()
        R = quat_to_R(*q)
        return p + TCP_OFF * R[:, 2], q

    def ik_hand(self, pos, quat, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state = JointState(name=ARM, position=list(seed))
        req.ik_request.timeout.sec = 3
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp(self, pos, quat, seed=None):
        R = quat_to_R(*quat)
        hand = np.array(pos, float) - TCP_OFF * R[:, 2]
        return self.ik_hand(hand, quat, seed)

    def move_joints(self, q, secs=3.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        cur = self.arm_q()
        err = max(abs(a - b) for a, b in zip(cur, q))
        return code, err

    def move_tcp(self, pos, yaw_deg=0.0, secs=3.0, quat=None):
        quat = quat or down_quat(yaw_deg)
        q = self.ik_tcp(pos, quat)
        if q is None:
            print(f"IK FAILED for tcp {pos}")
            return False
        # guard against wild joint flips: report the joint delta
        cur = self.arm_q()
        delta = [abs(a - b) for a, b in zip(cur, q)]
        code, err = self.move_joints(q, secs)
        # tolerance violations on long goals are usually controller lag;
        # resending the same goal converges (machine fact, docs/30-action.md)
        for _ in range(3):
            if code == 0 and err < 0.02:
                break
            print(f"  retry: code={code} err={err:.4f}")
            code, err = self.move_joints(q, max(secs, 2.0))
        t, _ = self.tcp()
        print(f"move_tcp -> {np.round(pos,3)} code={code} joint_err={err:.4f} "
              f"maxdelta={max(delta):.2f} tcp_now={np.round(t,3)}")
        return code == 0 and err < 0.02

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        self.spin(10, 0.1)
        gap = self.finger_gap()
        print(f"gripper({width}) reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap


if __name__ == "__main__":
    a = Arm()
    cmd = sys.argv[1]
    if cmd == "tcp":
        x, y, z = map(float, sys.argv[2:5])
        secs = float(sys.argv[5]) if len(sys.argv) > 5 else 3.0
        yaw = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
        ok = a.move_tcp((x, y, z), yaw, secs)
        sys.exit(0 if ok else 1)
    elif cmd == "grip":
        a.gripper(0.04 if sys.argv[2] == "open" else 0.0)
    elif cmd == "state":
        q = a.arm_q()
        print("joints", np.round(q, 3).tolist())
        p, o = a.fk(q)
        print("hand", np.round(p, 4).tolist(), np.round(o, 4).tolist())
        t, _ = a.tcp()
        print("tcp", np.round(t, 4).tolist())
        print("finger gap", round(a.finger_gap(), 4))
        print("wrench", a.wrench())
    rclpy.shutdown()
