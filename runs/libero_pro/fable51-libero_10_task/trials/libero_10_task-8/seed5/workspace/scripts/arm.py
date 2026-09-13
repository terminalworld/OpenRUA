#!/usr/bin/env python3
"""Small helper library for this Panda: joint state, FK/IK (world frame),
trajectory execution, gripper, servo bursts.

CLI:
  arm.py js                         print joint state + hand/TCP pose (FK)
  arm.py ik x y z qx qy qz qw       solve IK for TCP pose, print joints (no motion)
  arm.py goto x y z qx qy qz qw [sec]   IK for TCP pose, execute, verify
  arm.py joints p1,...,p7 [sec]     execute joint target, verify
  arm.py grip open|close            gripper
  arm.py servo dx dy dz n           n ticks of world-frame linear twist (m/s)
"""
import sys, time
from pathlib import Path

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

ROOT = Path(__file__).resolve().parents[1]
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
# hand orientation with fingers pointing down, opening along world X
Q_DOWN_X = (0.7071068, 0.7071068, 0.0, 0.0)
# fingers down, opening along world Y
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)


def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2, w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2, w1*w2 - x1*x2 - y1*y2 - z1*z2)


# machine fact: the IK service solves for panda_link8, which is yawed 45 deg
# from panda_hand about the shared z axis (same origin). Convert hand quats.
Q_HAND_TO_LINK8 = (0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8))


def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)

    def _on_js(self, m):
        self._js = m

    # ---------- sensing ----------
    def joint_state(self, fresh=True):
        if fresh:
            self._js = None
        end = time.time() + 30
        while self._js is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self._js is None:
            raise RuntimeError("no /joint_states")
        return dict(zip(self._js.name, self._js.position))

    def arm_joints(self, js=None):
        js = js or self.joint_state()
        return [js[j] for j in JOINTS]

    def finger_gap(self, js=None):
        js = js or self.joint_state(fresh=False)
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    def fk_pose(self, joints=None):
        joints = joints or self.arm_joints()
        self.fk.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(x) for x in joints]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        tcp = pos + TCP_OFF * quat_R(*q)[:, 2]
        return pos, q, tcp

    # ---------- planning ----------
    def solve_ik(self, tcp_xyz, q, seed=None):
        """IK for a TCP pose in world frame. Returns 7 joints or None."""
        R = quat_R(*q)
        hand = np.array(tcp_xyz, dtype=float) - TCP_OFF * R[:, 2]
        self.ik.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand)
        q8 = qmul(q, Q_HAND_TO_LINK8)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)
        req.ik_request.avoid_collisions = False
        seed = seed or self.arm_joints()
        req.ik_request.robot_state.joint_state.name = JOINTS
        req.ik_request.robot_state.joint_state.position = [float(x) for x in seed]
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print("IK failed", None if res is None else res.error_code.val)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    # ---------- acting ----------
    def move_joints(self, target, seconds=3.0, tol=0.02, retries=2):
        self.fjt.wait_for_server(timeout_sec=10)
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = JOINTS
            pt = JointTrajectoryPoint(positions=[float(x) for x in target])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            cur = self.arm_joints()
            err = max(abs(a - b) for a, b in zip(cur, target))
            print(f"traj error_code={code} max_joint_err={err:.4f}")
            if err < tol:
                return True
        return err < tol

    def goto_tcp(self, tcp_xyz, q, seconds=3.0):
        sol = self.solve_ik(tcp_xyz, q)
        if sol is None:
            return False
        ok = self.move_joints(sol, seconds)
        pos, qq, tcp = self.fk_pose()
        print(f"TCP now {tcp.round(4)} (target {np.round(tcp_xyz,4)}) ok={ok}")
        return ok

    def gripper(self, open_):
        self.grip.wait_for_server(timeout_sec=10)
        g = GripperCommand.Goal()
        g.command.position = float(GRIP["open_m"] if open_ else GRIP["closed_m"])
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        js = self.joint_state()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap(js):.4f}")
        return self.finger_gap(js)

    def servo(self, dx, dy, dz, ticks):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(dx), float(dy), float(dz)
        for _ in range(int(ticks)):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        stop = TwistStamped(); stop.header.frame_id = TWIST["frame"]
        for _ in range(3):
            self.twist_pub.publish(stop); rclpy.spin_once(self.node, timeout_sec=0.05)


def main():
    a = sys.argv[1:]
    arm = Arm()
    cmd = a[0]
    if cmd == "js":
        js = arm.joint_state()
        print({k: round(v, 4) for k, v in js.items()})
        pos, q, tcp = arm.fk_pose()
        print("hand", pos.round(4), "q", np.round(q, 4), "tcp", tcp.round(4), "gap", round(arm.finger_gap(js), 4))
    elif cmd == "ik":
        sol = arm.solve_ik([float(x) for x in a[1:4]], [float(x) for x in a[4:8]])
        print("sol", None if sol is None else ",".join(f"{x:.5f}" for x in sol))
        if sol:
            print("fk check", arm.fk_pose(sol)[2].round(4))
    elif cmd == "goto":
        sec = float(a[8]) if len(a) > 8 else 3.0
        arm.goto_tcp([float(x) for x in a[1:4]], [float(x) for x in a[4:8]], sec)
    elif cmd == "joints":
        sec = float(a[2]) if len(a) > 2 else 3.0
        arm.move_joints([float(x) for x in a[1].split(",")], sec)
        print("tcp", arm.fk_pose()[2].round(4))
    elif cmd == "grip":
        arm.gripper(a[1] == "open")
    elif cmd == "servo":
        arm.servo(float(a[1]), float(a[2]), float(a[3]), int(a[4]))
        print("tcp", arm.fk_pose()[2].round(4))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
