#!/usr/bin/env python3
"""Small manipulation helper for this Panda.

  arm.py fk                         hand + TCP pose in WORLD
  arm.py goto X Y Z YAWDEG [SECS]   move so the TCP (fingertip centre) is at
                                    world X Y Z, hand pointing down, fingers
                                    opening along world y rotated by YAWDEG
  arm.py grip open|close
World -> base offset comes from TF (world->panda_link0 = -0.51 0 0.42).
"""
import math, sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
BASE = np.array([0.0, 0.0, 0.0])  # IK/FK on this machine are already in WORLD (virtual_joint)
TCP = 0.1034
LIMITS = [(-2.9, 2.9), (-1.76, 1.76), (-2.9, 2.9), (-3.07, -0.07),
          (-2.9, 2.9), (-0.02, 3.75), (-2.9, 2.9)]


def quat_down(yaw_deg):
    # (1,0,0,0) = hand pointing down, fingers along world y; then yaw about world z
    s, c = math.sin(math.radians(yaw_deg) / 2), math.cos(math.radians(yaw_deg) / 2)
    # q = qz * (1,0,0,0)  ->  (c, s, 0, 0)
    return (c, s, 0.0, 0.0)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.update(zip(m.name, m.position)), 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fkc = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory,
                                 "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js.clear()
        while not all(j in self.js for j in ARM):
            self.spin(0.2)
        return [self.js[j] for j in ARM]

    def seed(self):
        s = JointState()
        s.name = list(ARM)
        s.position = self.joints()
        return s

    def fk(self, q=None):
        self.fkc.wait_for_service(5)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = q if q is not None else self.joints()
        fut = self.fkc.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise SystemExit(f"FK failed: {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        x, y, z, w = q
        # hand z axis in world (3rd column of R)
        zaxis = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
        return pos, q, pos + TCP * zaxis

    def solve_ik(self, tcp_world, yaw_deg):
        hand_world = np.array(tcp_world) + np.array([0, 0, TCP])   # hand points down
        p_base = hand_world - BASE
        qx, qy, qz, qw = quat_down(yaw_deg)
        self.ik.wait_for_service(5)
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"   # group tip is link8 (45 deg off)
        req.ik_request.pose_stamped.header.frame_id = ""
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, p_base)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = qx, qy, qz, qw
        req.ik_request.robot_state.joint_state = self.seed()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 5
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None:
            raise SystemExit("IK: no answer")
        if r.error_code.val != 1:
            raise SystemExit(f"IK FAILED code={r.error_code.val} for tcp={tcp_world} yaw={yaw_deg}")
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        q = [sol[j] for j in ARM]
        for v, (lo, hi) in zip(q, LIMITS):
            if not lo <= v <= hi:
                raise SystemExit(f"IK solution outside limits: {q}")
        return q

    def execute(self, q, secs):
        self.traj.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        now = np.array(self.joints())
        err = np.abs(now - np.array(q)).max()
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def goto(self, tcp_world, yaw_deg, secs=3.0):
        q = self.solve_ik(tcp_world, yaw_deg)
        print("IK q =", np.round(q, 3).tolist())
        code, err = self.execute(q, secs)
        if err > 0.02:
            print("large residual; resending")
            self.execute(q, secs)
        pos, quat, tcp = self.fk()
        print(f"hand world={np.round(pos,4).tolist()} TCP world={np.round(tcp,4).tolist()}")
        return tcp

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        self.js.clear()
        self.joints()
        while "panda_finger_joint1" not in self.js:
            self.spin(0.2)
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"finger1={self.js['panda_finger_joint1']:.4f}")
        return self.js["panda_finger_joint1"]


def main():
    a = sys.argv[1:]
    arm = Arm()
    if a[0] == "fk":
        pos, q, tcp = arm.fk()
        print("joints", np.round(arm.joints(), 4).tolist())
        print("hand world", np.round(pos, 4).tolist(), "quat", np.round(q, 3).tolist())
        print("TCP world", np.round(tcp, 4).tolist())
    elif a[0] == "goto":
        x, y, z, yaw = map(float, a[1:5])
        secs = float(a[5]) if len(a) > 5 else 3.0
        arm.goto((x, y, z), yaw, secs)
    elif a[0] == "ik":
        x, y, z, yaw = map(float, a[1:5])
        print(np.round(arm.solve_ik((x, y, z), yaw), 3).tolist())
    elif a[0] == "grip":
        arm.gripper(0.04 if a[1] == "open" else 0.0)
    elif a[0] == "fingers":
        arm.joints(); print(arm.js.get("panda_finger_joint1"))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
