#!/usr/bin/env python3
"""World-frame TCP mover for this Panda.

Usage:
  python3 tools/action/arm.py tcp <x> <y> <z> <yaw_deg> [seconds=4]
      top-down grasp pose: fingers axis = world y rotated by yaw_deg about z
  python3 tools/action/arm.py where
      print current hand + TCP pose in world (via TF)
Coordinates are WORLD frame; converted to panda_link0 for IK using the
world->panda_link0 TF. Verifies the reached pose from TF afterwards.
"""
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parents[2]
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
TCP_OFF = float(M["hand"]["tcp_offset_m"])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_tool")
        self.tf = Buffer()
        TransformListener(self.tf, self.node)
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, TRAJ["port"])

    def spin(self, t=0.2):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            self.spin()
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def lookup(self, parent, child):
        end = self.node.get_clock().now().nanoseconds / 1e9 + 10
        while not self.tf.can_transform(parent, child, Time()):
            self.spin()
            if self.node.get_clock().now().nanoseconds / 1e9 > end:
                raise SystemExit(f"no TF {parent}->{child}")
        t = self.tf.lookup_transform(parent, child, Time())
        tr = t.transform.translation
        q = t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), (q.x, q.y, q.z, q.w)

    def where(self):
        # spin a bit to refresh the TF buffer with fresh stamps
        for _ in range(10):
            self.spin(0.1)
        p, q = self.lookup("world", "panda_hand")
        R = quat_to_R(*q)
        tcp = p + TCP_OFF * R[:, 2]
        j = self.joints()
        print(f"hand world=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}) q=({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f})")
        print(f"tcp  world=({tcp[0]:.4f}, {tcp[1]:.4f}, {tcp[2]:.4f})")
        print("fingers:", j.get("panda_finger_joint1"), j.get("panda_finger_joint2"))
        return tcp

    def tcp(self, x, y, z, yaw_deg, seconds=4.0):
        th = math.radians(yaw_deg)
        q = (math.cos(th / 2), math.sin(th / 2), 0.0, 0.0)  # Rz(yaw)*Rx(180)
        R = quat_to_R(*q)
        hand_w = np.array([x, y, z]) - TCP_OFF * R[:, 2]
        # verified via /compute_fk: move_group's model frame IS world here
        # (panda_link0 reported at world (-0.51, 0, 0.42)), so pass world
        # coordinates straight through with an empty frame_id
        hand_b = hand_w
        if not self.ik.wait_for_service(timeout_sec=10):
            raise SystemExit("IK service unavailable")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"  # group tip is link8 (45deg off)
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        cur = self.joints()
        seed = JointState()
        for n in TRAJ["joints"]:
            seed.name.append(n)
            seed.position.append(cur[n])
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"IK FAILED code={None if res is None else res.error_code.val}; no motion")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        target = [sol[j] for j in TRAJ["joints"]]
        print("IK ok, target joints:", [round(v, 3) for v in target])
        for attempt in range(3):
            self.goto(target, seconds if attempt == 0 else 2.0)
            after = self.joints()
            err = max(abs(after[j] - t) for j, t in zip(TRAJ["joints"], target))
            print(f"max joint err after move: {err:.4f} rad")
            if err < 0.02:
                break
            print("resending (controller lag)")
        return self.where()

    def goto(self, target, seconds):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(TRAJ["joints"])
        pt = JointTrajectoryPoint(positions=[float(v) for v in target])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        print(f"trajectory done error_code={code}")


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    arm = Arm()
    if a[0] == "where":
        arm.where()
    elif a[0] == "tcp":
        x, y, z, yaw = map(float, a[1:5])
        secs = float(a[5]) if len(a) > 5 else 4.0
        arm.tcp(x, y, z, yaw, secs)
    else:
        raise SystemExit(__doc__)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
