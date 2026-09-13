#!/usr/bin/env python3
"""Shared helpers: joint state, FK, IK, trajectory, gripper. Import or run.

  python3 tools/robot.py state          # joints + hand pose (base frame)
  python3 tools/robot.py move j1,...,j7 [sec]
  python3 tools/robot.py grip open|close
  python3 tools/robot.py ik x y z qx qy qz qw [sec]   # plan via IK and move (TCP pose)
"""
import sys, time, math
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from moveit_msgs.msg import RobotState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from builtin_interfaces.msg import Duration

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP = M["hand"]["tcp_offset_m"]


def quat_to_mat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = np.argmax(np.diag(R))
        if i == 0:
            s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return np.array([x, y, z, w])


class Robot(Node):
    def __init__(self):
        super().__init__("robot_helper")
        self._js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fk_cli = self.create_client(GetPositionFK, M["planning"]["ik_service"].replace("ik", "fk"))
        self.ik_cli = self.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.traj_cli = ActionClient(self, FollowJointTrajectory, FJT["port"])
        self.grip_cli = ActionClient(self, GripperCommand, GRIP["port"])

    def _on_js(self, msg):
        self._js = msg

    # ---- sensing
    def joints(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in JOINTS])

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def _call(self, cli, req, timeout=60):
        while not cli.wait_for_service(timeout_sec=1.0):
            pass
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return fut.result()

    def fk(self, q=None, link="panda_hand"):
        """Return (pos, quat[xyzw], R) of link in panda_link0 frame."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        res = self._call(self.fk_cli, req)
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat, quat_to_mat(quat)

    def tcp(self, q=None):
        pos, quat, R = self.fk(q)
        return pos + R @ np.array([0, 0, TCP]), quat, R

    def ik(self, pos, quat, seed=None, at_tcp=True, attempts=5, avoid=False):
        """pos/quat of the TCP (or panda_hand if at_tcp False) in panda_link0. Returns q or None."""
        pos = np.asarray(pos, float)
        R = quat_to_mat(quat)
        if at_tcp:
            pos = pos - R @ np.array([0, 0, TCP])
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = bool(avoid)
        ps = PoseStamped()
        ps.header.frame_id = ""
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, pos)
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = map(float, quat)
        req.ik_request.pose_stamped = ps
        req.ik_request.timeout.sec = 2
        for _ in range(attempts):
            res = self._call(self.ik_cli, req)
            if res is not None and res.error_code.val == 1:
                d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                q = np.array([d[j] for j in JOINTS])
                if all(lo + 0.05 <= v <= hi - 0.05 for v, (lo, hi) in zip(q, LIMITS)):
                    return q
                print("ik: solution near joint limit, retrying", np.round(q, 2), flush=True)
            # perturb seed and retry
            req.ik_request.robot_state.joint_state.position = [
                float(v + np.random.uniform(-0.3, 0.3)) for v in seed]
        return None

    # ---- acting
    def move(self, q_list, seconds, wait=True):
        """q_list: one q (7,) or list of waypoints; seconds: total duration (evenly spaced)."""
        q_list = np.atleast_2d(np.asarray(q_list, float))
        for q in q_list:
            for i, (v, (lo, hi)) in enumerate(zip(q, LIMITS)):
                if v < lo - 1e-6 or v > hi + 1e-6:
                    raise ValueError(f"joint {i+1} value {v:.3f} outside [{lo},{hi}]")
        while not self.traj_cli.wait_for_server(timeout_sec=1.0):
            pass
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        n = len(q_list)
        for i, q in enumerate(q_list):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q]
            pt.velocities = [0.0] * 7 if (i == n - 1) else []
            t = seconds * (i + 1) / n
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            goal.trajectory.points.append(pt)
        hold = JointTrajectoryPoint(); hold.positions = [float(v) for v in q_list[-1]]; hold.velocities = [0.0] * 7
        th = seconds + 3.0
        hold.time_from_start = Duration(sec=int(th), nanosec=int((th - int(th)) * 1e9))
        goal.trajectory.points.append(hold)
        fut = self.traj_cli.send_goal_async(goal)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        gh = fut.result()
        if not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        if not wait:
            return gh
        rf = gh.get_result_async()
        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        res = rf.result().result
        err = np.abs(self.arm_q() - q_list[-1]).max()
        print(f"traj done: error_code={res.error_code} max_joint_err={err:.4f}", flush=True)
        return res.error_code, err

    def grip(self, open_=True, wait=True):
        while not self.grip_cli.wait_for_server(timeout_sec=1.0):
            pass
        goal = GripperCommand.Goal()
        goal.command.position = float(GRIP["open_m"] if open_ else GRIP["closed_m"])
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip_cli.send_goal_async(goal)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        gh = fut.result()
        rf = gh.get_result_async()
        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        r = rf.result().result
        print(f"grip {'open' if open_ else 'close'}: pos={r.position:.4f} reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}", flush=True)
        return r


def main():
    rclpy.init()
    r = Robot()
    cmd = sys.argv[1] if len(sys.argv) > 1 else "state"
    if cmd == "state":
        q = r.arm_q()
        print("q =", np.round(q, 4).tolist())
        print("fingers gap =", round(r.finger_gap(), 4))
        pos, quat, R = r.fk(q)
        print("hand pos =", np.round(pos, 4).tolist(), "quat xyzw =", np.round(quat, 4).tolist())
        tpos, _, _ = r.tcp(q)
        print("tcp  pos =", np.round(tpos, 4).tolist())
        print("hand R =\n", np.round(R, 3))
    elif cmd == "move":
        q = [float(v) for v in sys.argv[2].split(",")]
        sec = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
        r.move(q, sec)
        print("now q =", np.round(r.arm_q(), 4).tolist())
    elif cmd == "grip":
        r.grip(sys.argv[2] == "open")
    elif cmd == "ik":
        v = [float(x) for x in sys.argv[2:9]]
        sec = float(sys.argv[9]) if len(sys.argv) > 9 else 3.0
        q = r.ik(v[:3], v[3:])
        if q is None:
            print("IK FAILED"); sys.exit(1)
        print("ik q =", np.round(q, 4).tolist())
        r.move(q, sec)
        tpos, _, _ = r.tcp()
        print("tcp now =", np.round(tpos, 4).tolist())
    r.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
