#!/usr/bin/env python3
"""Mini command driver for the Panda: one node, reused clients.

Usage: python3 -u arm.py "<cmd>" ["<cmd>" ...]
  grip <per_finger_m>                      open/close gripper, report finger gap
  goto <x> <y> <z> [secs] [qx qy qz qw]    TCP to world pose (default top-down)
  fk                                       print current hand + TCP pose
  js                                       print joint state
Poses are in the planner model frame (== world here, verified by FK).
"""
import sys
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034
TOPDOWN = (1.0, 0.0, 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Arm:
    def __init__(self):
        self.node = rclpy.create_node("arm_driver")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fkc = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10) and self.grip.wait_for_server(10)
        assert self.ik.wait_for_service(10) and self.fkc.wait_for_service(10)
        self.spin_until(lambda: self.js is not None, 10)

    def _on_js(self, m):
        self.js = m

    def spin_until(self, pred, timeout):
        end = self.node.get_clock().now().nanoseconds / 1e9 + timeout
        while not pred() and self.node.get_clock().now().nanoseconds / 1e9 < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return pred()

    def fresh_js(self):
        old = self.js
        self.spin_until(lambda: self.js is not old, 5)
        return dict(zip(self.js.name, self.js.position))

    def arm_state(self):
        j = self.fresh_js()
        s = JointState()
        s.name = list(ARM)
        s.position = [j[n] for n in ARM]
        return s

    def call(self, cli, req, timeout=60):
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=timeout)
        return f.result()

    def fk_of(self, target):
        s = JointState()
        s.name = list(ARM)
        s.position = [float(v) for v in target]
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = s
        r = self.call(self.fkc, req)
        p, q = r.pose_stamped[0].pose.position, r.pose_stamped[0].pose.orientation
        return np.array([p.x, p.y, p.z]), (q.x, q.y, q.z, q.w)

    def fk(self):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_state()
        r = self.call(self.fkc, req)
        p, q = r.pose_stamped[0].pose.position, r.pose_stamped[0].pose.orientation
        R = quat_R(q.x, q.y, q.z, q.w)
        hand = np.array([p.x, p.y, p.z])
        tcp = hand + TCP * R[:, 2]
        return hand, tcp, (q.x, q.y, q.z, q.w)

    def goto(self, x, y, z, secs=4.0, quat=TOPDOWN):
        R = quat_R(*quat)
        hx, hy, hz = np.array([x, y, z]) - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = float(hx), float(hy), float(hz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self.arm_state()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        r = self.call(self.ik, req)
        if r is None or r.error_code.val != 1:
            print(f"IK FAILED for TCP ({x},{y},{z}): {None if r is None else r.error_code.val}")
            return False
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        target = [sol[n] for n in ARM]
        # the IK plugin returns solutions yawed about the hand axis; joint7 IS
        # that axis, so correct it in joint space before moving
        for _ in range(2):
            Rs = quat_R(*self.fk_of(target)[1])
            Rrel = Rs.T @ R
            yaw = float(np.arctan2(Rrel[1, 0], Rrel[0, 0]))
            if abs(yaw) < 0.01:
                break
            cand = target[6] + yaw
            if not (-2.85 < cand < 2.85):
                cand = target[6] + yaw - np.sign(yaw) * 2 * np.pi
            if not (-2.85 < cand < 2.85):
                print(f"  yaw fix {yaw:.3f} would exceed joint7 limit; leaving as is")
                break
            target[6] = cand
        ok = self.move(target, secs)
        hand, tcp, q = self.fk()
        err = np.linalg.norm(tcp - np.array([x, y, z]))
        print(f"  TCP now {tcp.round(4)} (err {err*1000:.1f} mm) q={np.round(q,3)}")
        return ok and err < 0.01

    def move(self, target, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in target])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        for attempt in range(2):
            f = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, f, timeout_sec=30)
            rf = f.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
            code = rf.result().result.error_code
            j = self.fresh_js()
            jerr = max(abs(j[n] - t) for n, t in zip(ARM, target))
            print(f"  traj error_code={code} max joint err={jerr:.4f} rad")
            if code == 0 and jerr < 0.02:
                return True
        return False

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = 30.0
        f = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=30)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        res = rf.result().result
        j = self.fresh_js()
        f1, f2 = j["panda_finger_joint1"], j["panda_finger_joint2"]
        print(f"  gripper reached={res.reached_goal} stalled={res.stalled} "
              f"fingers={f1:.4f},{f2:.4f} gap={f1 - f2:.4f}")
        return f1, f2


def main():
    import time
    for attempt in range(5):  # DDS peer resolution flakes right after another process exits
        try:
            rclpy.init()
            a = Arm()
            break
        except Exception as e:  # noqa
            print("node init retry:", str(e).splitlines()[0])
            try:
                rclpy.shutdown()
            except Exception:
                pass
            time.sleep(2)
    for cmd in sys.argv[1:]:
        print(">>", cmd, flush=True)
        t = cmd.split()
        if t[0] == "grip":
            a.gripper(float(t[1]))
        elif t[0] == "goto":
            x, y, z = map(float, t[1:4])
            secs = float(t[4]) if len(t) > 4 else 4.0
            quat = tuple(map(float, t[5:9])) if len(t) > 8 else TOPDOWN
            if not a.goto(x, y, z, secs, quat):
                print("!! goto did not converge; stopping sequence")
                break
        elif t[0] == "j7":  # rotate about the hand axis: joint7 += delta rad
            j = a.fresh_js()
            target = [j[n] for n in ARM]
            target[6] += float(t[1])
            a.move(target, float(t[2]) if len(t) > 2 else 2.0)
            hand, tcp, q = a.fk()
            print(f"  hand {hand.round(4)} tcp {tcp.round(4)} q {np.round(q,4)} "
                  f"fingerY={np.round(quat_R(*q)[:,1],3)}")
        elif t[0] == "fk":
            hand, tcp, q = a.fk()
            print(f"  hand {hand.round(4)} tcp {tcp.round(4)} q {np.round(q,4)}")
        elif t[0] == "js":
            j = a.fresh_js()
            print("  " + " ".join(f"{k}={v:.4f}" for k, v in j.items()))
        sys.stdout.flush()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
