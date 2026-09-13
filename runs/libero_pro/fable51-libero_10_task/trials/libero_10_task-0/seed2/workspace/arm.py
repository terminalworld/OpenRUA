#!/usr/bin/env python3
"""Persistent helper: IK + trajectory + gripper + joint-state reads.

World->base offset from TF (world -> panda_link0). Poses passed in WORLD
coordinates of the TCP (fingertip centre); converted to the planner's
base frame here.
"""
import sys, time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
# Verified: FK of the ready pose returns (-0.203, 0, 1.01) = (0.307, 0, 0.59)
# + world->panda_link0, so the planner's model frame IS world here.
BASE = np.array([0.0, 0.0, 0.0])
TOPDOWN = (1.0, 0.0, 0.0, 0.0)       # hand z down, fingers close along world y


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_quat(yaw):
    """Top-down grasp rotated by yaw about world z (yaw=0 -> fingers along y)."""
    # q = Rz(yaw) * (1,0,0,0)
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    # (c,0,0,s)*(1,0,0,0): w=c*0 - s*0..., do generic multiply
    w1, x1, y1, z1 = c, 0.0, 0.0, s
    w2, x2, y2, z2 = 0.0, 1.0, 0.0, 0.0
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 10)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fk.wait_for_service(10), "no FK"
        assert self.traj.wait_for_server(10), "no traj server"
        assert self.grip.wait_for_server(10), "no gripper server"
        self.joints()

    def _js(self, m):
        self.js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self):
        self.js = None
        while self.js is None:
            self.spin(0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def hand_pose(self):
        """FK of panda_hand in base frame -> world coords."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = self.arm_q()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        tcp = pos + TCP * quat_R(*q)[:, 2]
        return pos, tcp, q

    def solve_ik(self, tcp_world, quat=TOPDOWN, seed=None):
        R = quat_R(*quat)
        hand_world = np.array(tcp_world) - TCP * R[:, 2]
        hb = hand_world - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = seed or self.arm_q()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"IK FAILED for tcp={np.round(tcp_world,3)} code={None if r is None else r.error_code.val}")
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, q, seconds=3.0, tries=3):
        for attempt in range(tries):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ARM
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.traj.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            print(f"  traj error_code={code} max joint err={err:.4f} rad")
            if err < 0.006:
                return True
        return err < 0.02

    def move_tcp(self, tcp_world, quat=TOPDOWN, seconds=3.0):
        q = self.solve_ik(tcp_world, quat)
        if q is None:
            return False
        ok = self.move_joints(q, seconds)
        _, tcp, _ = self.hand_pose()
        print(f"  TCP now {np.round(tcp,3)} (target {np.round(tcp_world,3)}) err={np.linalg.norm(tcp-tcp_world):.4f}")
        return ok

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        for _ in range(5):
            self.spin(0.1)
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={np.round(f,4)}")
        return f

    def close(self):
        rclpy.shutdown()
