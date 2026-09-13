#!/usr/bin/env python3
"""Move the fingertip point (TCP) to world-frame targets, straight-down grasp
orientation. IK (base frame) -> FollowJointTrajectory per waypoint.

Usage: python3 move.py x,y,z[,yaw_deg][,seconds] [x,y,z[,yaw][,sec] ...]
       python3 move.py grip <per_finger_m>
       python3 move.py js
"""
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = TRAJ["joints"]
TCP = M["hand"]["tcp_offset_m"]
# machine fact (verified via FK/IK probe): IK with empty frame_id takes WORLD coords
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])


class Mover:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("mover")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, TRAJ["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fjt.wait_for_server(10), "no FJT"
        assert self.grip.wait_for_server(10), "no gripper"

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def solve(self, x, y, z, yaw_deg=0.0):
        """TCP world target -> arm joint dict. Hand points straight down,
        fingers along world y when yaw=0 (yaw rotates about world z)."""
        # hand z = -world z; hand origin sits TCP above the fingertip point
        hand_w = np.array([x, y, z + TCP]) - BASE_IN_WORLD
        # quaternion: Rz(yaw) * Rx(pi)  -> (cos(y/2), sin(y/2), 0, 0)*... compute
        half = np.deg2rad(yaw_deg) / 2
        # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sin,cos); product q = Rz*Rx:
        qx, qy, qz, qw = np.cos(half), np.sin(half), 0.0, 0.0
        cur = self.joints()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_w)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        seed = JointState()
        for j in JOINTS:
            seed.name.append(j)
            seed.position.append(cur[j])
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"IK failed for {x,y,z}: {None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def execute(self, positions, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        cur = self.joints()
        err = max(abs(cur[j] - p) for j, p in zip(JOINTS, positions))
        print(f"  traj error_code={code} max joint err={err:.4f}", flush=True)
        return code

    def goto(self, x, y, z, yaw=0.0, seconds=3.0):
        print(f"goto TCP world ({x:.3f},{y:.3f},{z:.3f}) yaw {yaw}", flush=True)
        q = self.solve(x, y, z, yaw)
        print("  ik:", np.round(q, 3).tolist(), flush=True)
        return self.execute(q, seconds)

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        j = self.joints()
        print(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={j['panda_finger_joint1']:.4f},{j['panda_finger_joint2']:.4f}", flush=True)


def main():
    mv = Mover()
    args = sys.argv[1:]
    if args and args[0] == "grip":
        mv.gripper(float(args[1]))
    elif args and args[0] == "js":
        print(mv.joints())
    else:
        for a in args:
            v = [float(s) for s in a.split(",")]
            x, y, z = v[:3]
            yaw = v[3] if len(v) > 3 else 0.0
            sec = v[4] if len(v) > 4 else 3.0
            mv.goto(x, y, z, yaw, sec)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
