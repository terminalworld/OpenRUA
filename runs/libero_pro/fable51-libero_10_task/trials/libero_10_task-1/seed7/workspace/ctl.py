#!/usr/bin/env python3
"""Small controller library for this Panda workstation (reusable clients).

Subcommands (all poses in the planner frame; see fk to learn which):
  fk                         print hand pose from current joints
  ik x y z qx qy qz qw       print IK solution (no motion)
  move x y z qx qy qz qw [sec] [--tcp]   IK + trajectory, then verify
  joints p1,...,p7 [sec]     trajectory to joint target
  grip open|close            gripper command, prints finger gap
  servo dx dy dz [n]         stream n twist ticks (m/s) in base frame
  js                         print joint state
"""
import sys
import time

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

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def js(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.js()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.js()
        return abs(j.get("panda_finger_joint1", 0)) + abs(j.get("panda_finger_joint2", 0))

    # ---- kinematics
    def fk(self, q=None, link="panda_hand"):
        q = q or self.arm_q()
        self.fk_cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]),
                r.pose_stamped[0].header.frame_id)

    def ik(self, pos, quat, seed=None, timeout=5.0):
        self.ik_cli.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(x) for x in (seed or self.arm_q())]
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=120)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None, (r.error_code.val if r else "timeout")
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM], 1

    # ---- motion
    def traj(self, q, sec=4.0, wait=True):
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        return code, err

    def move(self, pos, quat, sec=4.0, tcp=False):
        pos = np.array(pos, float)
        if tcp:
            pos = pos - TCP * quat_R(*quat)[:, 2]
        q, code = self.ik(pos, quat)
        if q is None:
            return None, f"IK failed {code}"
        code, err = self.traj(q, sec)
        p, _, _ = self.fk()
        return code, f"traj code={code} joint_err={err:.4f} hand_at={p.round(4)}"

    def gripper(self, width):
        self.grip.wait_for_server(timeout_sec=10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        send = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=180)
        r = res.result().result
        return r.reached_goal, r.stalled, self.finger_gap()

    def servo(self, v, n=20):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    r = Robot()
    cmd = a[0]
    if cmd == "js":
        j = r.js()
        print({k: round(v, 4) for k, v in j.items()})
    elif cmd == "fk":
        p, q, f = r.fk()
        print("frame", f, "pos", p.round(4), "quat", q.round(4))
    elif cmd == "ik":
        v = list(map(float, a[1:8]))
        print(r.ik(v[:3], v[3:]))
    elif cmd == "move":
        flags = [x for x in a if x.startswith("--")]
        v = [x for x in a[1:] if not x.startswith("--")]
        pos, quat = list(map(float, v[:3])), list(map(float, v[3:7]))
        sec = float(v[7]) if len(v) > 7 else 4.0
        print(r.move(pos, quat, sec, tcp="--tcp" in flags))
    elif cmd == "joints":
        q = list(map(float, a[1].split(",")))
        sec = float(a[2]) if len(a) > 2 else 4.0
        print(r.traj(q, sec))
    elif cmd == "grip":
        w = GRIP["open_m"] if a[1] == "open" else GRIP["closed_m"]
        print(r.gripper(w))
    elif cmd == "servo":
        v = list(map(float, a[1:4]))
        n = int(a[4]) if len(a) > 4 else 20
        r.servo(v, n)
        p, _, _ = r.fk()
        print("hand_at", p.round(4))
    r.node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
