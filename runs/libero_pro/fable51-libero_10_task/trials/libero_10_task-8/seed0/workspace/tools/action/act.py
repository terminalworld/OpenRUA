#!/usr/bin/env python3
"""World-frame motion helper (IK -> FollowJointTrajectory), reusable clients.

Usage:
  act.py pose                          print hand + TCP world pose from TF
  act.py move X Y Z YAWDEG [secs]      TCP to world (X,Y,Z), hand pointing
                                       down, fingers closing along world
                                       axis rotated YAWDEG from +y
  act.py line X Y Z YAWDEG [secs] [n]  straight TCP line from current pose
                                       to target in n IK waypoints
  act.py joints J1,...,J7 [secs]       raw joint target
"""
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parents[2]
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # FK/IK poses are already world-frame here (checked vs /compute_fk)
TCP = float(M["hand"]["tcp_offset_m"])


def quat_down(yaw_deg):
    """Hand z down; yaw_deg=0 -> hand y (closing) along world y."""
    psi = math.radians(yaw_deg)
    # q = qz(psi) * qx(pi)
    return (math.cos(psi / 2), math.sin(psi / 2), 0.0, 0.0)  # (x, y, z, w)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("act")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.update(zip(m.name, m.position)), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.buf = Buffer()
        TransformListener(self.buf, self.node)
        t0 = time.time()
        while time.time() - t0 < 10 and not all(j in self.js for j in JOINTS):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if not self.ik.wait_for_service(5) or not self.fjt.wait_for_server(5):
            raise SystemExit("IK service / FJT server missing")

    def spin(self, s=0.2):
        rclpy.spin_once(self.node, timeout_sec=s)

    def arm_q(self):
        return [self.js[j] for j in JOINTS]

    def hand_tf(self):
        t0 = time.time()
        while time.time() - t0 < 10:
            self.spin()
            if self.buf.can_transform("world", "panda_hand", rclpy.time.Time()):
                break
        t = self.buf.lookup_transform("world", "panda_hand", rclpy.time.Time()).transform
        p = np.array([t.translation.x, t.translation.y, t.translation.z])
        q = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        return p, q

    def tcp_pose(self):
        p, q = self.hand_tf()
        R = quat_to_R(*q)
        return p + TCP * R[:, 2], q

    def solve_ik(self, tcp_world, q, seed):
        R = quat_to_R(*q)
        hand_world = np.asarray(tcp_world) - TCP * R[:, 2]
        hb = hand_world - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.timeout.sec = 2
        req.ik_request.pose_stamped.header.frame_id = ""
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, hb)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = q
        req.ik_request.robot_state.joint_state.name = JOINTS
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"IK failed for {tcp_world}: "
                               f"{None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def send_traj(self, points, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        n = len(points)
        for i, q in enumerate(points):
            t = secs * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        # settle + fresh joint state
        for _ in range(5):
            self.spin(0.1)
        err = max(abs(a - b) for a, b in zip(self.arm_q(), points[-1]))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code

    def move(self, tcp, yaw, secs):
        q = quat_down(yaw)
        sol = self.solve_ik(tcp, q, self.arm_q())
        return self.send_traj([sol], secs)

    def line(self, tcp, yaw, secs, n):
        q = quat_down(yaw)
        p0, _ = self.tcp_pose()
        seed = self.arm_q()
        pts = []
        for i in range(1, n + 1):
            p = p0 + (np.asarray(tcp) - p0) * i / n
            seed = self.solve_ik(p, q, seed)
            pts.append(seed)
        return self.send_traj(pts, secs)

    def report(self):
        p, q = self.hand_tf()
        t, _ = self.tcp_pose()
        print(f"hand xyz={p.round(4)} quat(xyzw)={np.round(q, 4)}")
        print(f"tcp  xyz={t.round(4)}")
        print("fingers", round(self.js.get("panda_finger_joint1", -1), 4),
              round(self.js.get("panda_finger_joint2", -1), 4))


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    arm = Arm()
    cmd = a[0]
    if cmd == "pose":
        arm.report()
    elif cmd == "move":
        x, y, z, yaw = map(float, a[1:5])
        secs = float(a[5]) if len(a) > 5 else 3.0
        arm.move((x, y, z), yaw, secs)
        arm.report()
    elif cmd == "line":
        x, y, z, yaw = map(float, a[1:5])
        secs = float(a[5]) if len(a) > 5 else 2.0
        n = int(a[6]) if len(a) > 6 else 4
        arm.line((x, y, z), yaw, secs, n)
        arm.report()
    elif cmd == "joints":
        qs = [float(v) for v in a[1].split(",")]
        secs = float(a[2]) if len(a) > 2 else 3.0
        arm.send_traj([qs], secs)
        arm.report()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
