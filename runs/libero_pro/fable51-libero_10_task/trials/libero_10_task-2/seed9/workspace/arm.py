#!/usr/bin/env python3
"""Arm helper: IK (MoveIt) -> FollowJointTrajectory, gripper, FK, joint read.

Usage:
  python3 arm.py js                                   # joint state dict
  python3 arm.py fk                                   # hand + tcp pose (world)
  python3 arm.py tcp <x> <y> <z> <qx> <qy> <qz> <qw> [sec]   # world TCP pose -> move
  python3 arm.py hand <x> <y> <z> <qx> <qy> <qz> <qw> [sec]  # world HAND pose -> move
  python3 arm.py joints <p1,...,p7> [sec]             # raw joint target
  python3 arm.py j7 <delta_rad> [sec]                 # rotate wrist joint7 by delta
  python3 arm.py grip <width_m>                       # gripper per-finger position
"""
import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parent
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
PLAN = M["planning"]
JOINTS = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.660, 0.000, 0.912])  # from tf2_echo world panda_link0
# NOTE (verified empirically): /compute_fk AND /compute_ik both use WORLD-frame
# poses with frame_id="" on this machine (IK with base-frame coords fails -31).


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.update(zip(m.name, m.position)), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gripper = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, PLAN["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")

    def joints(self, fresh=True):
        if fresh:
            self._js.clear()
        while not all(j in self._js for j in JOINTS):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_state(self):
        js = self.joints()
        s = JointState()
        s.name = list(JOINTS)
        s.position = [js[j] for j in JOINTS]
        return s

    def fk_pose(self):
        self.fk.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_state()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])  # already world
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        tcp = pos + TCP_OFF * quat_to_R(*q)[:, 2]
        return pos, q, tcp

    def solve_ik(self, pos_world, q):
        self.ik.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = PLAN["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, np.asarray(pos_world))
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state = self.arm_state()
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise SystemExit("IK timeout")
        if res.error_code.val != 1:
            raise SystemExit(f"IK FAILED error_code={res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move_joints(self, positions, seconds):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        js = self.joints()
        err = max(abs(js[j] - p) for j, p in zip(JOINTS, positions))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_hand(self, pos_world, q, seconds):
        target = self.solve_ik(pos_world, q)
        print("IK ok ->", [round(v, 3) for v in target])
        return self.move_joints(target, seconds)

    def move_tcp(self, tcp_world, q, seconds):
        hand = np.asarray(tcp_world) - TCP_OFF * quat_to_R(*q)[:, 2]
        return self.move_hand(hand, q, seconds)

    def grip(self, width):
        if not self.gripper.wait_for_server(timeout_sec=10):
            raise SystemExit("no gripper server")
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        js = self.joints()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"f1={js.get('panda_finger_joint1'):.4f} f2={js.get('panda_finger_joint2'):.4f}")

    def report(self):
        pos, q, tcp = self.fk_pose()
        js = self.joints()
        print("joints:", [round(js[j], 4) for j in JOINTS],
              "fingers:", round(js.get("panda_finger_joint1", 0), 4),
              round(js.get("panda_finger_joint2", 0), 4))
        print("hand world:", np.round(pos, 4), "q:", np.round(q, 4))
        print("tcp  world:", np.round(tcp, 4))


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    arm = Arm()
    cmd = a[0]
    if cmd == "js":
        js = arm.joints()
        for k in sorted(js):
            print(f"{k}: {js[k]:.5f}")
    elif cmd == "fk":
        arm.report()
    elif cmd in ("tcp", "hand"):
        pos = [float(v) for v in a[1:4]]
        q = [float(v) for v in a[4:8]]
        sec = float(a[8]) if len(a) > 8 else 4.0
        (arm.move_tcp if cmd == "tcp" else arm.move_hand)(pos, q, sec)
        arm.report()
    elif cmd == "joints":
        pos = [float(v) for v in a[1].split(",")]
        sec = float(a[2]) if len(a) > 2 else 4.0
        arm.move_joints(pos, sec)
        arm.report()
    elif cmd == "j7":
        delta = float(a[1])
        sec = float(a[2]) if len(a) > 2 else 3.0
        js = arm.joints()
        pos = [js[j] for j in JOINTS]
        pos[6] += delta
        arm.move_joints(pos, sec)
        arm.report()
    elif cmd == "grip":
        arm.grip(float(a[1]))
    else:
        raise SystemExit(__doc__)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
