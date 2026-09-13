#!/usr/bin/env python3
"""Robot step runner. Each arg is one command; runs in order, stops on failure.

  move X Y Z [YAW_DEG] [SECS]   IK+trajectory; X Y Z = world TCP target (fingertips),
                                 hand pointing down; YAW = rotation of finger axis
                                 (0 -> fingers along world y, 90 -> along world x)
  grip open|close               gripper action; prints finger gap afterwards
  state                         joints, hand/TCP world pose, finger gap, wrench
"""
import sys, time
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as Rot
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GR = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


class Rob:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob")
        self.js = None
        self.wr = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 1)
        self.node.create_subscription(WrenchStamped, M["sensors"][1]["port"], self._wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GR["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10) and self.grip.wait_for_server(10)
        assert self.ik.wait_for_service(10) and self.fk.wait_for_service(10)

    def _js(self, m): self.js = m
    def _wr(self, m): self.wr = m

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js = None
            while self.js is None:
                self.spin()
        return dict(zip(self.js.name, self.js.position))

    def gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def hand_pose(self):
        j = self.joints()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = [j[a] for a in ARM]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        R = Rot.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, R

    def state(self):
        j = self.joints()
        pos, R = self.hand_pose()
        tcp = pos + TCP * R.as_matrix()[:, 2]
        print("arm:", [round(j[a], 4) for a in ARM])
        print("hand world:", pos.round(4), "TCP world:", tcp.round(4),
              "hand z-axis:", R.as_matrix()[:, 2].round(3))
        print("finger gap:", round(self.gap(), 4))
        if self.wr:
            f = self.wr.wrench.force
            print("wrench force:", round(f.x, 2), round(f.y, 2), round(f.z, 2))
        return tcp

    def solve_ik(self, tcp_xyz, yaw_deg):
        # hand pointing straight down; yaw rotates the finger axis about world z
        R = Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])
        hand = np.array(tcp_xyz) - TCP * R.as_matrix()[:, 2]
        q = R.as_quat()
        j = self.joints()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = [j[a] for a in ARM]
        req.ik_request.timeout.sec = 3
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print("IK FAILED", None if r is None else r.error_code.val)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[a] for a in ARM]

    def traj(self, positions, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        j = self.joints()
        err = max(abs(j[a] - p) for a, p in zip(ARM, positions))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code == 0 and err < 0.02

    def move(self, x, y, z, yaw=0.0, secs=3.0):
        sol = self.solve_ik([x, y, z], yaw)
        if sol is None:
            return False
        ok = self.traj(sol, secs)
        tcp = self.state()
        d = np.linalg.norm(tcp - np.array([x, y, z]))
        print(f"move -> TCP off-target by {d*1000:.1f} mm")
        return ok and d < 0.01

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GR["max_effort"])
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        for _ in range(5):
            self.spin(0.1)
        print(f"gripper -> reached={r.reached_goal} stalled={r.stalled} gap={self.gap():.4f}")
        return True


def main():
    r = Rob()
    for cmd in sys.argv[1:]:
        parts = cmd.split()
        print(f"\n== {cmd}")
        if parts[0] == "move":
            ok = r.move(*map(float, parts[1:]))
        elif parts[0] == "grip":
            ok = r.gripper(GR["open_m"] if parts[1] == "open" else GR["closed_m"])
        elif parts[0] == "state":
            r.state(); ok = True
        else:
            raise SystemExit(f"unknown {cmd}")
        if not ok:
            print("STEP FAILED, stopping")
            break
    rclpy.shutdown()


if __name__ == "__main__":
    main()
