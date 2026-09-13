#!/usr/bin/env python3
"""IK to a WORLD-frame TCP pose, then send the trajectory until it converges.

Usage: python3 goto.py <x> <y> <z> <qx> <qy> <qz> <qw> [seconds=4] [--joints j1,...,j7]
Resends the same joint goal (up to 4 times) while any joint is more than
TOL rad from target, since this controller lags long goals. Prints the
final TCP pose in world (via /compute_fk) so the caller can verify.
"""
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, "/workspace/tools/action")
from ik_move import _quat_to_R  # noqa: E402

TCP = 0.1034
TOL = 0.02
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]


class Robot:
    def __init__(self):
        self.node = rclpy.create_node("goto")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10)

    def _on_js(self, m):
        self.js = dict(zip(m.name, m.position))

    def joints(self):
        self.js = {}
        while not self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return [self.js[j] for j in JOINTS]

    def seed(self):
        s = JointState()
        s.name = list(JOINTS)
        s.position = self.joints()
        return s

    def solve(self, x, y, z, qx, qy, qz, qw):
        R = _quat_to_R(qx, qy, qz, qw)
        hx, hy, hz = np.array([x, y, z]) - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = hx, hy, hz
        p.orientation.x, p.orientation.y = qx, qy
        p.orientation.z, p.orientation.w = qz, qw
        req.ik_request.robot_state.joint_state = self.seed()
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"IK FAILED ({None if res is None else res.error_code.val})")
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def send(self, target, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(v) for v in target])
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        f = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, f)
        r = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, r)
        return r.result().result.error_code

    def move(self, target, seconds, tries=4):
        for i in range(tries):
            code = self.send(target, seconds)
            cur = self.joints()
            err = np.abs(np.array(cur) - np.array(target)).max()
            print(f"  try {i+1}: error_code={code} max_joint_err={err:.4f}",
                  flush=True)
            if err < TOL:
                return True
            seconds = max(seconds, 4) + 2
        return False

    def tcp(self):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.seed()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        q = p.orientation
        R = _quat_to_R(q.x, q.y, q.z, q.w)
        hand = np.array([p.position.x, p.position.y, p.position.z])
        return hand + TCP * R[:, 2], (q.x, q.y, q.z, q.w), R[:, 1]


def main():
    argv = sys.argv[1:]
    joints = None
    if "--joints" in argv:
        i = argv.index("--joints")
        joints = [float(v) for v in argv[i + 1].split(",")]
        del argv[i:i + 2]
    rclpy.init()
    rb = Robot()
    if joints is None:
        x, y, z, qx, qy, qz, qw = map(float, argv[:7])
        seconds = float(argv[7]) if len(argv) > 7 else 4.0
        joints = rb.solve(x, y, z, qx, qy, qz, qw)
        print("IK:", ",".join(f"{v:.4f}" for v in joints))
    else:
        seconds = float(argv[0]) if argv else 4.0
    ok = rb.move(joints, seconds)
    tcp, q, fa = rb.tcp()
    print(f"converged={ok} tcp_world {tcp[0]:.4f} {tcp[1]:.4f} {tcp[2]:.4f} "
          f"quat {np.round(q,3)} finger_axis {np.round(fa,3)}")
    print("fingers", round(rb.js.get("panda_finger_joint1", 0), 4),
          round(rb.js.get("panda_finger_joint2", 0), 4))
    rclpy.shutdown()
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
