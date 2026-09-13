#!/usr/bin/env python3
"""Move the TCP (fingertip centre) to a WORLD-frame pose: IK -> trajectory -> FK check.

Usage: python3 move_tcp.py <x> <y> <z> <qx> <qy> <qz> <qw> [seconds=4]
       python3 move_tcp.py fk            # just print the current TCP pose (world)
World -> planner base frame handled here (base at world BASE_P).
"""
import sys, time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_P = np.array([0.0, 0.0, 0.0])  # verified: /compute_fk & /compute_ik use the WORLD frame here
M = yaml.safe_load(open(Path(__file__).parent / "machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("move_tcp")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.traj.wait_for_server(10)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_state(self):
        j = self.joints()
        s = JointState(); s.name = list(ARM); s.position = [j[n] for n in ARM]
        return s

    def fk_tcp(self):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_state()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        R = quat_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        hand_w = np.array([p.position.x, p.position.y, p.position.z]) + BASE_P
        tcp_w = hand_w + TCP * R[:, 2]
        return tcp_w, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def move(self, x, y, z, qx, qy, qz, qw, secs=4.0):
        R = quat_R(qx, qy, qz, qw)
        hand_w = np.array([x, y, z]) - TCP * R[:, 2]
        hand_b = hand_w - BASE_P
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        # IK solves for panda_link8, which is panda_hand yawed by +45 deg
        # about the shared z axis (verified via FK); convert hand quat -> link8 quat
        from scipy.spatial.transform import Rotation as Rot
        q8 = (Rot.from_quat([qx, qy, qz, qw]) * Rot.from_euler("z", np.pi / 4)).as_quat()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)
        req.ik_request.robot_state.joint_state = self.arm_state()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print("IK FAILED", None if res is None else res.error_code.val)
            return False
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        target = [sol[j] for j in ARM]
        print("IK ok, joints:", np.round(target, 3).tolist())
        return self.move_joints(target, secs)

    def move_joints(self, target, secs, tries=4):
        # the controller lags on long goals (tolerance violation -5); resending
        # the same goal converges, so retry until the joints actually match
        for _ in range(tries):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(ARM)
            pt = JointTrajectoryPoint(positions=[float(v) for v in target])
            pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.traj.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            j = self.joints()
            err = max(abs(j[n] - t) for n, t in zip(ARM, target))
            print(f"traj error_code={code} max_joint_err={err:.4f}")
            if err < 0.02:
                return True
            secs = max(secs, 2.0)
        return False


def main():
    c = Ctl()
    if sys.argv[1] == "fk":
        pass
    else:
        vals = list(map(float, sys.argv[1:]))
        secs = vals[7] if len(vals) > 7 else 4.0
        c.move(*vals[:7], secs=secs)
    tcp, q = c.fk_tcp()
    print("TCP world:", np.round(tcp, 4).tolist(), "hand quat:", np.round(q, 4).tolist())
    j = c.joints()
    print("fingers:", round(j["panda_finger_joint1"], 4), round(j["panda_finger_joint2"], 4))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
