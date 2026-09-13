#!/usr/bin/env python3
"""Control helpers for the Panda: joint state, FK, IK, trajectories, gripper.

World frame <-> base frame is a pure translation (measured via TF):
    base = world - WORLD_T_BASE
Poses passed to move_tcp() are the TCP (fingertip point) in WORLD frame.
"""
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parent
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = TRAJ["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
# measured: MoveIt FK/IK poses (frame_id "") already match TF world frame
WORLD_T_BASE = np.array([0.0, 0.0, 0.0])

# hand pointing straight down, fingers along world X (180deg about (1,1,0)/sqrt2)
Q_DOWN_FINGERS_X = (0.70710678, 0.70710678, 0.0, 0.0)
# hand pointing straight down, fingers along world Y (180deg about X)
Q_DOWN_FINGERS_Y = (1.0, 0.0, 0.0, 0.0)


def quat_mul(a, b):
    """Hamilton product of xyzw quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


# measured: the IK service solves for panda_link8, whose frame is the hand
# frame rotated -45deg about z. Request hand*Rz(+45deg) to get the hand there.
Q_IK_FIX = (0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8))


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self, name="ctl"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, TRAJ["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        assert self.fjt.wait_for_server(timeout_sec=20), "no FJT server"
        assert self.grip.wait_for_server(timeout_sec=20), "no gripper server"
        assert self.ik.wait_for_service(timeout_sec=20), "no IK"
        assert self.fk.wait_for_service(timeout_sec=20), "no FK"

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js = {}
        end = time.time() + 20
        while not self._js and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in ARM]

    def finger_gap(self):
        js = self.joints()
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    def fk_hand(self, q=None):
        """Hand frame pose in WORLD: (pos(3), quat xyzw(4))."""
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + WORLD_T_BASE
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, quat

    def fk_tcp(self, q=None):
        pos, quat = self.fk_hand(q)
        R = quat_to_R(*quat)
        return pos + TCP_OFF * R[:, 2], quat

    # ---------- planning ----------
    def ik_tcp(self, pos_world, quat, seed=None, tries=1):
        """IK for a TCP pose in world. Returns arm joint list or None."""
        R = quat_to_R(*quat)
        hand_world = np.asarray(pos_world, float) - TCP_OFF * R[:, 2]
        hand_base = hand_world - WORLD_T_BASE
        seed = self.arm_q() if seed is None else seed
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, hand_base)
            qreq = quat_mul(quat, Q_IK_FIX)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, qreq)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
            print(f"IK failed code={None if res is None else res.error_code.val}", file=sys.stderr)
        return None

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, via=None):
        """Send a joint trajectory (optionally through 'via' waypoints)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = list(via or []) + [q]
        for i, wp in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        assert gh is not None and gh.accepted, "goal rejected"
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = float(np.max(np.abs(np.array(self.arm_q()) - np.array(q))))
        print(f"move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik_tcp(pos_world, quat, seed=seed)
        if q is None:
            print("move_tcp: IK failed, no motion", file=sys.stderr)
            return None
        self.move_q(q, seconds)
        pos, _ = self.fk_tcp()
        print(f"move_tcp: tcp now {np.round(pos, 4)} (target {np.round(pos_world, 4)})")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        res = rf.result().result
        gap = self.finger_gap()
        print(f"gripper({width}): reached={res.reached_goal} stalled={res.stalled} gap={gap:.4f}")
        return gap

    def servo(self, vx=0.0, vy=0.0, vz=0.0, ticks=20):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)


if __name__ == "__main__":
    r = Robot()
    print("joints", r.joints())
    pos, quat = r.fk_hand()
    print("hand world", pos, quat)
    print("tcp world", r.fk_tcp()[0])
    print("finger gap", r.finger_gap())
