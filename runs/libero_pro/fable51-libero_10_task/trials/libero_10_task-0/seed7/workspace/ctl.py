#!/usr/bin/env python3
"""Small controller CLI for the Panda (world-frame targets).

  python3 ctl.py fk                         # hand + tcp pose in world
  python3 ctl.py move X Y Z [yaw_deg] [T]   # TCP to world pose, top-down grasp orientation
  python3 ctl.py joints p1,...,p7 [T]       # raw joint trajectory
  python3 ctl.py grip open|close
  python3 ctl.py js                         # joint state
World->base offset: base = world - (-0.51, 0, 0.42)  (from TF world->panda_link0).
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
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = ARM["joints"]
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # FK check: planner model frame == world here
TCP = float(M["hand"]["tcp_offset_m"])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def topdown_quat(yaw_deg):
    """Hand Z down; yaw rotates the finger-opening axis about world Z.
    yaw=0 -> fingers open along world Y."""
    # q = Rz(yaw) * Rx(pi)
    h = math.radians(yaw_deg) / 2
    qz = (0, 0, math.sin(h), math.cos(h))
    qx = (1.0, 0, 0, 0)
    # quaternion multiply qz * qx
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, ARM["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fkc = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        end = time.time() + 30
        while "m" not in self._js and time.time() < end:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_state(self):
        js = self.joints()
        s = JointState()
        for j in JOINTS:
            s.name.append(j)
            s.position.append(js[j])
        return s, js

    def fk(self):
        self.fkc.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state, js = self.arm_state()
        fut = self.fkc.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        R = quat_to_R(*q)
        tcp = pos + TCP * R[:, 2]
        return pos + BASE_IN_WORLD, tcp + BASE_IN_WORLD, q, R, js

    def ik_solve(self, tcp_world, quat):
        self.ik.wait_for_service(10)
        R = quat_to_R(*quat)
        hand_base = np.array(tcp_world) - TCP * R[:, 2] - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state, _ = self.arm_state()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK failed: {None if res is None else res.error_code.val}")
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def traj(self, positions, seconds):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        result = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, result)
        code = result.result().result.error_code
        js = self.joints()
        err = max(abs(js[j] - p) for j, p in zip(JOINTS, positions))
        print(f"traj done error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move(self, tcp_world, yaw_deg=0.0, seconds=3.0):
        q = topdown_quat(yaw_deg)
        sol = self.ik_solve(tcp_world, q)
        if sol is None:
            return False
        code, err = self.traj(sol, seconds)
        hand, tcp, _, _, _ = self.fk()
        print(f"tcp now world=({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f}) "
              f"target=({tcp_world[0]:.3f},{tcp_world[1]:.3f},{tcp_world[2]:.3f})")
        return code == 0

    def grip(self, what):
        self.gr.wait_for_server(10)
        goal = GripperCommand.Goal()
        goal.command.position = float(GRIP["open_m"] if what == "open" else GRIP["closed_m"])
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=180)
        r = rf.result().result
        js = self.joints()
        f1, f2 = js["panda_finger_joint1"], js["panda_finger_joint2"]
        print(f"grip {what}: reached={r.reached_goal} stalled={r.stalled} "
              f"fingers=({f1:.4f},{f2:.4f}) gap={f1 - f2:.4f}")
        return f1, f2


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    c = Ctl()
    cmd = a[0]
    if cmd == "fk":
        hand, tcp, q, R, js = c.fk()
        print("hand world", np.round(hand, 4))
        print("tcp  world", np.round(tcp, 4))
        print("quat xyzw", np.round(q, 4))
        print("R", np.round(R, 3))
        print("fingers", js["panda_finger_joint1"], js["panda_finger_joint2"])
    elif cmd == "js":
        print(c.joints())
    elif cmd == "move":
        x, y, z = map(float, a[1:4])
        yaw = float(a[4]) if len(a) > 4 else 0.0
        T = float(a[5]) if len(a) > 5 else 3.0
        ok = c.move((x, y, z), yaw, T)
        sys.exit(0 if ok else 1)
    elif cmd == "joints":
        pos = [float(v) for v in a[1].split(",")]
        T = float(a[2]) if len(a) > 2 else 3.0
        c.traj(pos, T)
    elif cmd == "grip":
        c.grip(a[1])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
