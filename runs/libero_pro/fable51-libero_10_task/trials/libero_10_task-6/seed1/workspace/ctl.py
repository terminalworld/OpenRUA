#!/usr/bin/env python3
"""Sequenced TCP moves for the Panda, world-frame in, verified out.

Usage: python3 ctl.py CMD [CMD ...]
  move:x,y,z[,yaw_deg],secs   IK (base frame) -> trajectory; TCP target in WORLD
  grip:open|close             gripper to 0.04 / 0.0 per finger
  pose                        print TCP world pose + finger gap
Hand points straight down; yaw=0 => fingers open along world Y.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
# MoveIt's model frame here is WORLD (FK puts panda_link0 at -0.51,0,0.42);
# the panda_arm group's tip link is panda_link8 = panda_hand rotated -45deg about Z.
Q_HAND_TO_L8 = (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8))


def qmul(a, b):  # (x, y, z, w) Hamilton product a*b
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def down_quat(yaw_deg):
    """Hand Z down, hand X rotated by yaw about world Z. q = Rz(yaw) * Rx(pi)."""
    h = math.radians(yaw_deg) / 2
    # Rz(yaw) = (0,0,sin h,cos h); Rx(pi) = (1,0,0,0); product (Hamilton):
    return (math.cos(h), math.sin(h), 0.0, 0.0)  # (x, y, z, w)


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.update(zip(m.name, m.position)), 1)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10)
        self.fjt.wait_for_server(10)
        self.gr.wait_for_server(10)
        while not self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def spin(self, n=5):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def pose(self):
        self.spin(10)
        end = time.time() + 10
        while time.time() < end and not self.tfbuf.can_transform("world", "panda_hand", rclpy.time.Time()):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        t = self.tfbuf.lookup_transform("world", "panda_hand", rclpy.time.Time())
        q = t.transform.rotation
        R = quat_R(q.x, q.y, q.z, q.w)
        hand = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        tcp = hand + TCP * R[:, 2]
        gap = self.js.get("panda_finger_joint1", float("nan")) - self.js.get("panda_finger_joint2", float("nan"))
        print(f"TCP world {tcp.round(4)} hand z-axis {R[:,2].round(3)} hand y-axis {R[:,1].round(3)} "
              f"finger gap {gap:.4f}")
        return tcp

    def move(self, x, y, z, yaw, secs):
        qx, qy, qz, qw = down_quat(yaw)
        R = quat_R(qx, qy, qz, qw)
        hand_b = np.array([x, y, z]) - TCP * R[:, 2]  # link8 origin == hand origin
        qx, qy, qz, qw = qmul((qx, qy, qz, qw), Q_HAND_TO_L8)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        seed = JointState()
        self.spin(5)
        for j in ARM:
            seed.name.append(j)
            seed.position.append(float(self.js[j]))
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK FAILED for TCP ({x},{y},{z}) yaw {yaw}: "
                  f"{None if res is None else res.error_code.val}")
            return False
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        target = [sol[j] for j in ARM]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=target)
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        rf = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf)
        code = rf.result().result.error_code
        self.spin(10)
        err = max(abs(self.js[j] - t) for j, t in zip(ARM, target))
        print(f"move -> TCP ({x},{y},{z}) yaw {yaw}: fjt code {code}, max joint err {err:.4f}")
        self.pose()
        return code == 0

    def grip(self, what):
        goal = GripperCommand.Goal()
        goal.command.position = GRIP["open_m"] if what == "open" else GRIP["closed_m"]
        goal.command.max_effort = float(GRIP["max_effort"])
        send = self.gr.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30)
        rf = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        self.spin(10)
        gap = self.js["panda_finger_joint1"] - self.js["panda_finger_joint2"]
        print(f"grip {what}: reached={r.reached_goal} stalled={r.stalled} gap {gap:.4f}")


def main():
    c = Ctl()
    for cmd in sys.argv[1:]:
        if cmd == "pose":
            c.pose()
        elif cmd.startswith("grip:"):
            c.grip(cmd.split(":")[1])
        elif cmd.startswith("move:"):
            v = [float(s) for s in cmd.split(":")[1].split(",")]
            if len(v) == 4:
                x, y, z, secs = v; yaw = 0.0
            else:
                x, y, z, yaw, secs = v
            if not c.move(x, y, z, yaw, secs):
                print("ABORTING sequence")
                break
    rclpy.shutdown()


if __name__ == "__main__":
    main()
