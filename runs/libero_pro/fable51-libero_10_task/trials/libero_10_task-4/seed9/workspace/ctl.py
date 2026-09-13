#!/usr/bin/env python3
"""Persistent-node arm controller helpers (IK/FK/trajectory/gripper/state).

Usage (CLI):
  ctl.py state                      # joints + hand/tcp pose (world)
  ctl.py fk                         # FK of current joints
  ctl.py tcp x y z qx qy qz qw [sec] # move TCP to world pose (IK+FJT)
  ctl.py grip open|close
  ctl.py wrench
"""
import sys, time
from pathlib import Path
import numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK

ROOT = Path(__file__).resolve().parent
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])   # verified: IK/FK with empty frame_id are in WORLD on this machine
DOWN_X = (0.7071068, 0.7071068, 0.0, 0.0)      # hand down, fingers close along world x
DOWN_Y = (1.0, 0.0, 0.0, 0.0)                  # hand down, fingers close along world y


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Ctl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("ctl")
        self.js = None
        self.wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, next(s for s in M["sensors"] if s["kind"] == "wrench")["port"], self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.spin_until(lambda: self.js is not None, 15)

    def _on_js(self, m): self.js = m
    def _on_wr(self, m): self.wr = m

    def spin_until(self, pred, timeout):
        end = time.time() + timeout
        while not pred() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return pred()

    def joints(self):
        self.js = None
        self.spin_until(lambda: self.js is not None, 10)
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in ARM], d.get("panda_finger_joint1"), d

    def wrench(self):
        self.wr = None
        self.spin_until(lambda: self.wr is not None, 10)
        f = self.wr.wrench.force
        return np.array([f.x, f.y, f.z])

    # ---- kinematics -------------------------------------------------
    def fk_hand(self, q=None):
        if q is None:
            q = self.joints()[0]
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = list(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        tcp = pos + TCP_OFF * quat_R(*quat)[:, 2]
        return pos, quat, tcp

    def ik_hand(self, pos_world, quat, seed=None):
        """IK for HAND frame pose in world. Returns joint list or None."""
        self.ik.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        b = np.asarray(pos_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        if seed is None:
            seed = self.joints()[0]
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = list(map(float, seed))
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"IK failed code={r and r.error_code.val} for {pos_world} {quat}")
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp(self, tcp_world, quat, seed=None):
        hand = np.asarray(tcp_world) - TCP_OFF * quat_R(*quat)[:, 2]
        return self.ik_hand(hand, quat, seed)

    # ---- motion -----------------------------------------------------
    def move_joints(self, q, seconds=3.0, retries=2):
        self.fjt.wait_for_server(10)
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(ARM)
            pt = JointTrajectoryPoint(positions=list(map(float, q)))
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            gh = send.result()
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            cur = np.array(self.joints()[0])
            err = np.abs(cur - np.array(q)).max()
            print(f"  fjt code={code} max_joint_err={err:.4f}")
            if err < 0.02:
                return True
            seconds = max(1.5, seconds * 0.7)
        return err < 0.05

    def move_tcp(self, tcp_world, quat, seconds=3.0, seed=None):
        q = self.ik_tcp(tcp_world, quat, seed)
        if q is None:
            return False
        ok = self.move_joints(q, seconds)
        pos, qt, tcp = self.fk_hand()
        print(f"  tcp now {tcp.round(4)} (target {np.asarray(tcp_world).round(4)})")
        return ok

    def gripper(self, open_):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(GRIP["open_m"] if open_ else GRIP["closed_m"])
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        gap = self.joints()[1]
        print(f"  gripper {'open' if open_ else 'close'}: reached={r.reached_goal} stalled={r.stalled} finger={gap:.4f}")
        return gap


def main():
    c = Ctl()
    cmd = sys.argv[1]
    if cmd == "state":
        q, f, _ = c.joints()
        print("joints", np.round(q, 4).tolist(), "finger", round(f, 4))
        pos, quat, tcp = c.fk_hand(q)
        print("hand", pos.round(4), np.round(quat, 4), "tcp", tcp.round(4))
    elif cmd == "tcp":
        v = list(map(float, sys.argv[2:9]))
        sec = float(sys.argv[9]) if len(sys.argv) > 9 else 3.0
        print(c.move_tcp(v[:3], v[3:7], sec))
    elif cmd == "grip":
        c.gripper(sys.argv[2] == "open")
    elif cmd == "wrench":
        print(c.wrench())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
