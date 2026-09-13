#!/usr/bin/env python3
"""Arm helper: IK/FK/trajectory/gripper with clients built once.

  arm.py ik x y z qx qy qz qw            -> print joint solution (planner frame coords)
  arm.py go x y z qx qy qz qw [sec] [--tcp] -> IK then execute; verify via joint state
  arm.py joints p1,...,p7 [sec]          -> execute joint target
  arm.py grip <per-finger m>
  arm.py fk
"""
import sys, time
import numpy as np, rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from px import q2R

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034


class Arm:
    def __init__(self):
        rclpy.init(); self.node = rclpy.create_node("arm")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip_cli = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")

    def _on_js(self, m):
        self.js = dict(zip(m.name, m.position))

    def joints(self, fresh=True):
        if fresh:
            self.js = {}
        end = time.time() + 15
        while not self.js and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self.js)

    def arm_pos(self):
        j = self.joints(); return [j[n] for n in ARM]

    def fk(self):
        self.fk_cli.wait_for_service(10)
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = self.arm_pos()
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        hand = np.array([p.position.x, p.position.y, p.position.z])
        R = q2R(p.orientation)
        return hand, hand + TCP * R[:, 2], R, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik(self, x, y, z, qx, qy, qz, qw, seed=None):
        self.ik_cli.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, (qx, qy, qz, qw))
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = seed or self.arm_pos()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 5
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print("IK failed", None if r is None else r.error_code.val); return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move(self, target, sec=3.0, verify=True):
        self.traj.wait_for_server(10)
        g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(v) for v in target])
        pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
        g.trajectory.points = [pt]
        fut = self.traj.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        cur = self.arm_pos()
        err = max(abs(a - b) for a, b in zip(cur, target))
        print(f"move done code={code} max_joint_err={err:.4f}")
        return code, err

    def grip(self, w):
        self.grip_cli.wait_for_server(10)
        g = GripperCommand.Goal(); g.command.position = float(w); g.command.max_effort = 30.0
        fut = self.grip_cli.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        j = self.joints()
        gap = j["panda_finger_joint1"] - j["panda_finger_joint2"]
        print(f"grip done reached={r.reached_goal} stalled={r.stalled} f1={j['panda_finger_joint1']:.4f} f2={j['panda_finger_joint2']:.4f} gap={gap:.4f}")
        return gap

    def go(self, x, y, z, q, sec=3.0, tcp=False):
        """q is the desired panda_hand orientation (xyzw, world). The IK tip is
        panda_link8 = hand rotated +45 deg about its z, so convert before asking."""
        R = q2R(type("Q", (), dict(x=q[0], y=q[1], z=q[2], w=q[3]))())
        if tcp:
            x, y, z = np.array([x, y, z]) - TCP * R[:, 2]
        c, s = np.cos(np.pi / 8), np.sin(np.pi / 8)
        qx, qy, qz, qw = q
        q8 = (qw * 0 + qx * c + qy * s, qy * c - qx * s, qz * c + qw * s, qw * c - qz * s)
        sol = self.ik(x, y, z, *q8)
        if sol is None:
            return False
        print("ik sol:", [round(v, 4) for v in sol])
        code, err = self.move(sol, sec)
        hand, tcpp, R, qq = self.fk()
        print("now hand:", hand.round(4), "tcp:", tcpp.round(4), "quat:", [round(v, 3) for v in qq])
        return code == 0 and err < 0.02


def main():
    a = Arm(); cmd = sys.argv[1]; args = [v for v in sys.argv[2:] if not v.startswith("--")]
    if cmd == "fk":
        hand, tcp, R, q = a.fk(); print("hand", hand.round(4), "tcp", tcp.round(4), "quat", [round(v, 4) for v in q])
        print("axes x", R[:, 0].round(3), "y", R[:, 1].round(3), "z", R[:, 2].round(3))
        j = a.joints(); print("fingers", j["panda_finger_joint1"], j["panda_finger_joint2"])
    elif cmd == "ik":
        print(a.ik(*map(float, args[:7])))
    elif cmd == "go":
        v = list(map(float, args)); sec = v[7] if len(v) > 7 else 3.0
        print("OK" if a.go(v[0], v[1], v[2], v[3:7], sec, tcp="--tcp" in sys.argv) else "FAIL")
    elif cmd == "joints":
        a.move([float(v) for v in args[0].split(",")], float(args[1]) if len(args) > 1 else 3.0)
    elif cmd == "grip":
        a.grip(float(args[0]))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
