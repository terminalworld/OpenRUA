#!/usr/bin/env python3
"""Persistent-client helper: run a sequence of commands in one process.

Usage: python3 -u robot.py "<cmd>" "<cmd>" ...
Commands (world-frame metres; TCP = fingertip point; top-down grasp):
  goto X Y Z [yaw=0] [secs=3]   IK to TCP pose, then trajectory; verifies
  grip open|close               gripper action, prints finger gap after
  pose                          FK -> TCP world pose + finger gap
  joints                        print arm joint positions
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
# verified empirically: /compute_fk and /compute_ik on this machine work
# in the WORLD frame (FK of panda_link0 = (-0.51, 0, 0.42)), so no offset
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10), "no fjt server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.ik.wait_for_service(10), "no ik"
        assert self.fk.wait_for_service(10), "no fk"
        self.wait_js()

    def _on_js(self, m):
        self.js = m

    def wait_js(self):
        self.js = None
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(zip(self.js.name, self.js.position))

    def arm_state(self):
        d = self.wait_js()
        s = JointState()
        s.name = list(ARM)
        s.position = [d[j] for j in ARM]
        return s, d

    def finger_gap(self, d):
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def call(self, cli, req, timeout=60):
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=timeout)
        return f.result()

    def tcp_pose(self):
        seed, d = self.arm_state()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = seed
        res = self.call(self.fk, req)
        p = res.pose_stamped[0].pose
        R = quat_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        hand = np.array([p.position.x, p.position.y, p.position.z])
        tcp = hand + TCP * R[:, 2]
        return tcp + BASE_IN_WORLD, R, d

    def goto(self, x, y, z, yaw=0.0, secs=3.0):
        q = (math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0)  # Rz(yaw)*Rx(pi)
        R = quat_R(*q)
        tcp_b = np.array([x, y, z]) - BASE_IN_WORLD
        hand_b = tcp_b - TCP * R[:, 2]
        seed, _ = self.arm_state()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        res = self.call(self.ik, req)
        if res is None or res.error_code.val != 1:
            print(f"IK FAILED code={None if res is None else res.error_code.val}")
            return False
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        target = [sol[j] for j in ARM]
        cur = [seed.position[i] for i in range(len(ARM))]
        jump = max(abs(a - b) for a, b in zip(target, cur))
        print(f"IK ok, max joint jump {jump:.2f} rad")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=target)
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        t0 = time.time()
        f = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code if rf.result() else "timeout"
        _, d = self.arm_state()
        err = max(abs(d[j] - t) for j, t in zip(ARM, target))
        tcp, _, _ = self.tcp_pose()
        print(f"traj code={code} ({time.time()-t0:.0f}s) joint err {err:.3f} "
              f"TCP world {tcp.round(3)}")
        return True

    def gripper(self, open_):
        goal = GripperCommand.Goal()
        goal.command.position = GRIP["open_m"] if open_ else GRIP["closed_m"]
        goal.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result if rf.result() else None
        _, d = self.arm_state()
        print(f"gripper {'open' if open_ else 'close'}: reached={getattr(r,'reached_goal',None)} "
              f"stalled={getattr(r,'stalled',None)} gap={self.finger_gap(d):.4f}")


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    r = Robot()
    for cmd in sys.argv[1:]:
        parts = cmd.split()
        print(f">>> {cmd}", flush=True)
        if parts[0] == "goto":
            args = [float(v) for v in parts[1:]]
            x, y, z = args[:3]
            yaw = args[3] if len(args) > 3 else 0.0
            secs = args[4] if len(args) > 4 else 3.0
            if not r.goto(x, y, z, yaw, secs):
                print("ABORT sequence"); break
        elif parts[0] == "grip":
            r.gripper(parts[1] == "open")
        elif parts[0] == "pose":
            tcp, R, d = r.tcp_pose()
            print(f"TCP world {tcp.round(4)} hand z-axis {R[:,2].round(3)} "
                  f"gap={r.finger_gap(d):.4f}")
        elif parts[0] == "joints":
            _, d = r.arm_state()
            print({j: round(d[j], 4) for j in ARM})
        sys.stdout.flush()
    print("DONE", flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
