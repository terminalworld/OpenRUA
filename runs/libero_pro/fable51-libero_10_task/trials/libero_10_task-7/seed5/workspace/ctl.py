#!/usr/bin/env python3
"""Small controller: chain commands in one process (clients built once).

Usage: python3 ctl.py "cmd args; cmd args; ..."
  tcp X Y Z [secs]      move so the TCP (fingertip point) is at world XYZ,
                        hand pointing straight down, fingers along world Y
  tcpyaw X Y Z YAW [s]  same, with a yaw (rad) about world Z
  grip W                per-finger position (0.04 open, 0.0 closed)
  pose                  print world hand/TCP pose + finger gap
  js                    print arm joint positions
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
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.zeros(3)  # verified via /compute_fk: empty frame_id == world on this machine


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.buf = Buffer()
        TransformListener(self.buf, self.node)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        assert self.fjt.wait_for_server(10) and self.grip.wait_for_server(10)
        assert self.ik.wait_for_service(10)

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            self.spin()
        m = self.js["m"]
        d = dict(zip(m.name, m.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def pose(self):
        end = time.time() + 10
        while time.time() < end and not self.buf.can_transform("world", "panda_hand", rclpy.time.Time()):
            self.spin(0.2)
        t = self.buf.lookup_transform("world", "panda_hand", rclpy.time.Time()).transform
        q = t.rotation
        R = quat_R(q.x, q.y, q.z, q.w)
        hand = np.array([t.translation.x, t.translation.y, t.translation.z])
        tcp = hand + TCP * R[:, 2]
        d = self.joints()
        gap = d.get("panda_finger_joint1", float("nan"))
        print(f"hand {hand.round(4)} tcp {tcp.round(4)} quat ({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f}) finger1 {gap:.4f}")
        return hand, tcp, R

    def solve_ik(self, xyz_world, quat, seed_q=None, verbose=True):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        p = req.ik_request.pose_stamped.pose
        xyz = np.array(xyz_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = JointState()
        seed.name = list(ARM)
        seed.position = [float(v) for v in (seed_q if seed_q is not None else self.arm_q())]
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            if verbose:
                print("IK FAILED", None if res is None else res.error_code.val)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    HOME = np.array([0, -0.161, 0, -2.44, 0, 2.23, 0.785])

    def best_ik(self, hand, quat):
        """Try several seeds; prefer solutions that stay out of the
        physically self-colliding fold (j4 < -2.8 with j6 > 2.9) and
        close to the current configuration."""
        cur = np.array(self.arm_q())
        seeds = [cur, self.HOME, self.HOME + [0.3, 0, -0.3, 0, 0, 0, 0],
                 self.HOME + [-0.3, 0, 0.3, 0, 0, 0, 0], self.HOME + [0, 0, 0, 0, 0.5, 0, 0]]
        best, best_cost = None, 1e9
        for s in seeds:
            q = self.solve_ik(hand, quat, s, verbose=False)
            if q is None:
                continue
            cost = np.abs(q - cur).sum()
            if q[3] < -2.8:
                cost += 10 * (-2.8 - q[3])
            if q[5] > 2.85:
                cost += 10 * (q[5] - 2.85)
            if abs(q[6]) > 2.5 or abs(q[0]) > 2.0:
                cost += 5
            if cost < best_cost:
                best, best_cost = q, cost
        if best is None:
            print("IK FAILED for all seeds")
        else:
            print("IK ok:", best.round(3), "cost %.2f" % best_cost)
        return best

    def move_q(self, q, secs, retries=2):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        for attempt in range(retries + 1):
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
            print(f"traj error_code={code} max joint err after={err:.4f}")
            if err < 0.02:
                return 0  # converged (a -5 with tiny residual is controller lag)
            if attempt < retries:
                print("  residual too large, resending same goal")
        return code if code != 0 else -99

    def tcp(self, x, y, z, pitch_deg=0.0, yaw_deg=0.0, secs=3.0):
        """Hand pointing down; pitch tilts the approach axis toward -x
        (wrist moves away from the base), yaw rotates the finger axis
        (0 = fingers close along world Y)."""
        from scipy.spatial.transform import Rotation as Rot
        R = (Rot.from_euler("z", yaw_deg, degrees=True).as_matrix()
             @ Rot.from_euler("y", pitch_deg, degrees=True).as_matrix()
             @ quat_R(1, 0, 0, 0))
        quat = Rot.from_matrix(R).as_quat()
        hand = np.array([x, y, z]) - TCP * R[:, 2]
        q = self.best_ik(hand, quat)
        if q is None:
            return 99
        code = self.move_q(q, secs)
        self.pose()
        return code

    def gripper(self, w):
        goal = GripperCommand.Goal()
        goal.command.position = float(w)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        d = self.joints()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} pos={r.position:.4f} finger1={d['panda_finger_joint1']:.4f}")


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


FORCE = "--force" in sys.argv  # keep going after a tolerance-violation result


def main():
    c = Ctl()
    for cmd in sys.argv[1].split(";"):
        parts = cmd.split()
        if not parts:
            continue
        op, a = parts[0], [float(x) for x in parts[1:]]
        print(">>", cmd.strip(), flush=True)
        if op == "tcp":  # tcp X Y Z [pitch_deg] [yaw_deg] [secs]
            code = c.tcp(*a[:3], *(a[3:6] + [0.0, 0.0, 3.0][len(a) - 3:]))
            if code != 0 and not FORCE:
                print("STOPPING chain"); break
        elif op == "grip":
            c.gripper(a[0])
        elif op == "pose":
            c.pose()
        elif op == "js":
            print(np.array(c.arm_q()).round(4))
        else:
            print("unknown", op)
        sys.stdout.flush()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
