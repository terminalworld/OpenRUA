#!/usr/bin/env python3
"""World-frame arm helpers for this Panda (base at world (-0.51,0,0.42)).

CLI:
  python3 arm.py go <x> <y> <z> [yaw_deg] [seconds]   # HAND frame, top-down
  python3 arm.py tcp <x> <y> <z> [yaw_deg] [seconds]  # fingertip point
  python3 arm.py grip <per_finger_m>
  python3 arm.py state                                # hand pose + fingers
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
# Verified: FK/IK on this machine are expressed in WORLD (FK of the hand
# matches TF world->panda_hand; IK with a base offset fails), so no offset.
BASE_W = np.array([0.0, 0.0, 0.0])


def topdown_quat(yaw_deg):
    """Hand Z down; fingers close along world axis rotated yaw from +y."""
    # q = qz(yaw) * (1,0,0,0)
    h = math.radians(yaw_deg) / 2
    qz = (0, 0, math.sin(h), math.cos(h))
    qx = (1.0, 0.0, 0.0, 0.0)
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_seed(self):
        j = self.joints()
        s = JointState()
        for n in JOINTS:
            s.name.append(n); s.position.append(j[n])
        return s, j

    def hand_pose_world(self):
        seed, j = self.arm_seed()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = seed
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_W
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), j

    def solve_ik(self, xw, yw, zw, yaw_deg, at_tcp=False, seed=None):
        q = topdown_quat(yaw_deg)
        p = np.array([xw, yw, zw]) - BASE_W
        if at_tcp:
            p = p + np.array([0, 0, TCP])  # hand is TCP m above fingertips
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, p)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = q
        if seed is None:
            req.ik_request.robot_state.joint_state, _ = self.arm_seed()
        else:
            js = JointState(); js.name = list(JOINTS); js.position = [float(v) for v in seed]
            req.ik_request.robot_state.joint_state = js
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            code = None if res is None else res.error_code.val
            raise RuntimeError(f"IK failed code={code} for {xw,yw,zw,yaw_deg}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def send_traj(self, points, seconds):
        """points: list of joint vectors; evenly spaced in time."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        n = len(points)
        for i, p in enumerate(points):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in p])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        j = self.joints()
        err = max(abs(j[n] - v) for n, v in zip(JOINTS, points[-1]))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        # machine fact: tolerance violations on long goals are controller
        # lag; resend the final point until the joints converge
        tries = 0
        while err > 0.01 and tries < 4:
            tries += 1
            goal.trajectory.points = [JointTrajectoryPoint(
                positions=[float(v) for v in points[-1]],
                time_from_start=Duration(sec=2))]
            send = self.traj.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            j = self.joints()
            err = max(abs(j[n] - v) for n, v in zip(JOINTS, points[-1]))
            print(f"  retry {tries}: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def go(self, xw, yw, zw, yaw_deg=0.0, seconds=3.0, at_tcp=False):
        sol = self.solve_ik(xw, yw, zw, yaw_deg, at_tcp)
        code, err = self.send_traj([sol], seconds)
        pos, _, _ = self.hand_pose_world()
        print(f"hand now at world {pos.round(4)}")
        return pos

    def line(self, p0, p1, yaw_deg, seconds, n=4, at_tcp=True):
        """Straight-ish TCP line p0->p1 as n IK waypoints, each seeded by
        the previous so the arm stays on one branch."""
        _, _, j = self.hand_pose_world()
        seed = [j[k] for k in JOINTS]
        pts = []
        for i in range(1, n + 1):
            p = np.array(p0) + (np.array(p1) - np.array(p0)) * i / n
            seed = self.solve_ik(*p, yaw_deg, at_tcp=at_tcp, seed=seed)
            pts.append(seed)
        code, err = self.send_traj(pts, seconds)
        pos, _, _ = self.hand_pose_world()
        print(f"hand now at world {pos.round(4)} (tcp z {pos[2]-TCP:.4f})")
        return pos

    def tcp(self):
        pos, q, j = self.hand_pose_world()
        return pos - np.array([0, 0, TCP])

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        j = self.joints()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={j['panda_finger_joint1']:.4f},{j['panda_finger_joint2']:.4f}")
        return j

    def close(self):
        self.node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    a = Arm()
    cmd = sys.argv[1]
    try:
        if cmd in ("go", "tcp"):
            x, y, z = map(float, sys.argv[2:5])
            yaw = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
            sec = float(sys.argv[6]) if len(sys.argv) > 6 else 3.0
            a.go(x, y, z, yaw, sec, at_tcp=(cmd == "tcp"))
        elif cmd == "grip":
            a.gripper(float(sys.argv[2]))
        elif cmd == "state":
            pos, q, j = a.hand_pose_world()
            print("hand world", pos.round(4), "quat", np.round(q, 3))
            print("fingers", j["panda_finger_joint1"], j["panda_finger_joint2"])
            print("arm", [round(j[n], 4) for n in JOINTS])
    finally:
        a.close()
