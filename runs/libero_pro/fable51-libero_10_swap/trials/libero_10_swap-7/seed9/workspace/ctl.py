#!/usr/bin/env python3
"""Reusable controller: joint state, FK, IK, trajectory, gripper for the Panda.

World frame -> panda_link0 frame: base sits at world (-0.51, 0, 0.42).
TCP targets are given in WORLD coordinates with the hand pointing down.
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
ARM = FJT["joints"]
# Verified empirically: /compute_fk and /compute_ik on this machine work in
# the WORLD frame (model root is `world`), so no base offset is applied.
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])
TCP_OFF = float(M["hand"]["tcp_offset_m"])


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def down_quat(yaw=0.0):
    """Hand Z pointing down (-world z), fingers separate along world y when yaw=0.
    q = Rz(yaw) * Rx(pi)."""
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sin(yaw/2),cos(yaw/2))
    cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    # quaternion product Rz * Rx : (w1,x1,y1,z1)*(w2,x2,y2,z2)
    w1, x1, y1, z1 = cz, 0.0, 0.0, sz
    w2, x2, y2, z2 = 0.0, 1.0, 0.0, 0.0
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


class Ctl:
    def __init__(self, name="ctl"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 10)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.traj.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.joints()

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))

    def spin(self, t=0.3):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self._js = {}
        while not self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in ARM]

    def finger_gap(self):
        js = self.joints()
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    def _seed(self, q=None):
        q = q if q is not None else self.arm_q()
        s = JointState()
        s.name = list(ARM)
        s.position = [float(v) for v in q]
        return s

    def fk_hand(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]),
                r.pose_stamped[0].header.frame_id, r.error_code.val)

    def tcp_world(self, q=None):
        """World position of the fingertip point (hand + TCP_OFF along hand z)."""
        p, quat, frame, code = self.fk_hand(q)
        x, y, z, w = quat
        zaxis = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
        return p + TCP_OFF * zaxis + BASE_IN_WORLD, quat

    def ik_tcp_world(self, xyz, yaw=0.0, seed=None, tries=5):
        """IK for TCP at world xyz with hand pointing down. Returns arm q or None."""
        qx, qy, qz, qw = down_quat(yaw)
        R = quat_R(qx, qy, qz, qw)
        hand = np.array(xyz, dtype=float) - TCP_OFF * R[:, 2] - BASE_IN_WORLD
        for i in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, hand)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
            req.ik_request.robot_state.joint_state = self._seed(seed)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                q = [sol[j] for j in ARM]
                return q
            log(f"IK attempt {i} failed code={None if r is None else r.error_code.val}")
            # perturb seed
            base = seed if seed is not None else self.arm_q()
            seed = list(np.array(base) + np.random.uniform(-0.3, 0.3, 7))
        return None

    def move_q(self, q, secs=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = secs * (i + 1) / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        fut = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        res = rf.result()
        code = res.result.error_code if res else None
        self.spin(0.3)
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        log(f"move done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, xyz, yaw=0.0, secs=3.0, seed=None):
        q = self.ik_tcp_world(xyz, yaw, seed)
        if q is None:
            log(f"IK FAILED for {xyz}")
            return None
        code, err = self.move_q(q, secs)
        tcp, _ = self.tcp_world()
        log(f"tcp now {np.round(tcp, 4)} target {np.round(xyz, 4)}")
        return q

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        res = rf.result()
        self.spin(0.5)
        gap = self.finger_gap()
        log(f"gripper({width}) reached={res.result.reached_goal if res else None} "
            f"stalled={res.result.stalled if res else None} gap={gap:.4f}")
        return gap


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


if __name__ == "__main__":
    c = Ctl("ctl_cli")
    cmd = sys.argv[1]
    if cmd == "fk":
        p, q, f, code = c.fk_hand()
        print("hand in", f, p, q, code)
        print("tcp world", c.tcp_world()[0])
        print("joints", c.joints())
    elif cmd == "ik":
        print(c.ik_tcp_world([float(v) for v in sys.argv[2:5]],
                             float(sys.argv[5]) if len(sys.argv) > 5 else 0.0))
    elif cmd == "go":
        c.move_tcp([float(v) for v in sys.argv[2:5]],
                   float(sys.argv[5]) if len(sys.argv) > 5 else 0.0,
                   float(sys.argv[6]) if len(sys.argv) > 6 else 3.0)
    elif cmd == "grip":
        c.gripper(float(sys.argv[2]))
    elif cmd == "q":
        c.move_q([float(v) for v in sys.argv[2].split(",")],
                 float(sys.argv[3]) if len(sys.argv) > 3 else 3.0)
