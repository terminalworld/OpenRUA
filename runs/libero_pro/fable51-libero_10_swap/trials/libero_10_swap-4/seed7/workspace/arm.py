#!/usr/bin/env python3
"""Small helper for this Panda: world-frame TCP moves via IK + FJT.

Usage:
  python3 arm.py pose                       # current TCP pose (world)
  python3 arm.py goto x,y,z[,secs] [x,y,z[,secs] ...]
        # TCP waypoints in world frame, hand pointing down; each
        # waypoint is one IK + one trajectory, seeded from the last
  python3 arm.py grip open|close
Options: --yaw <deg> rotate the hand about world z (0 = fingers along y)
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
# verified live: /compute_fk and /compute_ik on this machine both speak
# the WORLD frame (frame_id left empty), so no base offset is needed
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])


def quat_down(yaw_deg=0.0):
    """Hand z pointing down (-world z); yaw about world z."""
    # base: 180deg about x -> (1,0,0,0). Then yaw about z: q_z * q_x
    h = math.radians(yaw_deg) / 2
    qz = np.array([0, 0, math.sin(h), math.cos(h)])  # x y z w
    qx = np.array([1.0, 0, 0, 0])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.wait_js()

    def _on_js(self, m):
        self.js = m

    def wait_js(self):
        self.js = None
        t0 = time.time()
        while self.js is None and time.time() - t0 < 10:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self.js is None:
            raise SystemExit("no /joint_states")
        return dict(zip(self.js.name, self.js.position))

    def arm_state(self):
        d = self.wait_js()
        s = JointState()
        for j in JOINTS:
            s.name.append(j)
            s.position.append(d[j])
        return s

    def fingers(self):
        d = self.wait_js()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def tcp_pose(self):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_state()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise SystemExit(f"FK failed: {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        q = p.orientation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        hand = np.array([p.position.x, p.position.y, p.position.z])
        tcp = hand + TCP * R[:, 2] + BASE_IN_WORLD
        return tcp, np.array([q.x, q.y, q.z, q.w])

    def solve_ik(self, tcp_world, quat, seed=None):
        q = np.asarray(quat, float)
        R = quat_to_R(*q)
        hand = np.asarray(tcp_world, float) - TCP * R[:, 2] - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = hand
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        req.ik_request.robot_state.joint_state = seed or self.arm_state()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None:
            raise SystemExit("IK no answer")
        if r.error_code.val != 1:
            raise SystemExit(f"IK failed code={r.error_code.val} for tcp "
                             f"{np.round(tcp_world, 3)}")
        sol = dict(zip(r.solution.joint_state.name,
                       r.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move_joints(self, positions, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(secs),
                                      nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        # verify
        d = self.wait_js()
        err = max(abs(d[j] - p) for j, p in zip(JOINTS, positions))
        print(f"  traj code={code} max joint err={err:.4f} rad")
        return code, err

    def goto(self, tcp_world, quat, secs=3.0):
        # machine quirk (verified by FK): /compute_ik applies the requested
        # orientation to panda_link8, which sits 45deg about z from
        # panda_hand; pre-rotate so the HAND ends up at `quat`
        q = np.asarray(quat, float)
        h = math.radians(-45.0) / 2
        x1, y1, z1, w1 = 0.0, 0.0, math.sin(h), math.cos(h)
        x2, y2, z2, w2 = q
        q_req = np.array([
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ])
        sol = self.solve_ik(tcp_world, q_req)
        code, err = self.move_joints(sol, secs)
        if err > 0.02:
            print("  large joint error, resending")
            code, err = self.move_joints(sol, secs)
        tcp, q = self.tcp_pose()
        d = np.linalg.norm(tcp - np.asarray(tcp_world))
        print(f"  TCP now {np.round(tcp, 4)} (target {np.round(tcp_world, 4)}"
              f", off {d*1000:.1f} mm) quat {np.round(q, 3)}")
        return tcp

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={f[0]:.4f},{f[1]:.4f}")
        return f


def main():
    args = sys.argv[1:]
    yaw = 0.0
    if "--yaw" in args:
        i = args.index("--yaw")
        yaw = float(args[i + 1])
        del args[i:i + 2]
    if not args:
        raise SystemExit(__doc__)
    arm = Arm()
    cmd = args[0]
    if cmd == "pose":
        tcp, q = arm.tcp_pose()
        print(f"TCP world {np.round(tcp, 4)} quat {np.round(q, 3)} "
              f"fingers {arm.fingers()}")
    elif cmd == "goto":
        q = quat_down(yaw)
        for wp in args[1:]:
            v = [float(x) for x in wp.split(",")]
            secs = v[3] if len(v) > 3 else 3.0
            print(f"goto {v[:3]} in {secs}s")
            arm.goto(v[:3], q, secs)
    elif cmd == "grip":
        arm.gripper(GRIP["open_m"] if args[1] == "open" else GRIP["closed_m"])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
