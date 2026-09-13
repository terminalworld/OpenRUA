#!/usr/bin/env python3
"""Small controller library for the Panda: IK/FK, trajectories, gripper,
servo bursts, joint-state reads. Import or run as a CLI:

  python3 ctl.py js                         # joint state
  python3 ctl.py fk                         # hand pose (from current joints)
  python3 ctl.py ik x y z qx qy qz qw       # IK only, print joints
  python3 ctl.py move x y z qx qy qz qw [sec] [--tcp]   # IK + trajectory
  python3 ctl.py joints p1,...,p7 [sec]
  python3 ctl.py grip open|close
  python3 ctl.py servo dx dy dz [n_ticks]   # m/s in base frame, ~20 Hz
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
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Ctl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grp = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw = self.node.create_publisher(TwistStamped, TW["port"], 10)

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))
        self._js_eff = dict(zip(msg.name, msg.effort))

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def js(self, fresh=True):
        if fresh:
            self._js = {}
        while not self._js:
            self.spin(0.2)
        return dict(self._js)

    def arm_q(self):
        j = self.js()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.js()
        return abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"])

    # ---- kinematics --------------------------------------------------
    def _seed(self, q=None):
        q = q if q is not None else self.arm_q()
        s = JointState(); s.name = list(ARM); s.position = [float(v) for v in q]
        return s

    def fk_pose(self, q=None, link="panda_hand"):
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]),
                r.pose_stamped[0].header.frame_id)

    def ik_q(self, pos, quat, seed=None, attempts=3):
        self.ik.wait_for_service(10)
        for _ in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            # the group's default tip is panda_link8 (45 deg yawed from the
            # hand); solve for the hand frame explicitly
            req.ik_request.ik_link_name = "panda_hand"
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            (p.orientation.x, p.orientation.y,
             p.orientation.z, p.orientation.w) = map(float, quat)
            req.ik_request.robot_state.joint_state = self._seed(seed)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name,
                               r.solution.joint_state.position))
                return [sol[j] for j in ARM]
            code = None if r is None else r.error_code.val
        raise RuntimeError(f"IK failed (code {code}) for {pos}")

    # ---- motion ------------------------------------------------------
    def move_joints(self, q, seconds=3.0, via=None):
        """One trajectory through optional via points then q."""
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = list(via or []) + [q]
        n = len(pts)
        for i, p in enumerate(pts):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in p])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_pose(self, pos, quat, seconds=3.0, tcp=False, seed=None):
        pos = np.array(pos, float)
        if tcp:
            pos = pos - TCP_OFF * quat_R(*quat)[:, 2]
        q = self.ik_q(pos, quat, seed=seed)
        return q, self.move_joints(q, seconds)

    def grip(self, what):
        self.grp.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(GRIP["open_m"] if what == "open" else GRIP["closed_m"])
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grp.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        gap = self.finger_gap()
        print(f"grip {what}: reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap

    def servo(self, v, ticks=20):
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(ticks):
            self.tw.publish(msg); self.spin(0.05)


def main():
    a = sys.argv[1:]
    c = Ctl()
    cmd = a[0]
    if cmd == "js":
        print(c.js())
    elif cmd == "fk":
        p, q, f = c.fk_pose()
        print("frame", f, "pos", p.round(4), "quat", q.round(4))
    elif cmd == "ik":
        print(c.ik_q(list(map(float, a[1:4])), list(map(float, a[4:8]))))
    elif cmd == "move":
        nums = [float(x) for x in a[1:] if not x.startswith("--")]
        sec = nums[7] if len(nums) > 7 else 3.0
        c.move_pose(nums[0:3], nums[3:7], sec, tcp="--tcp" in a)
    elif cmd == "joints":
        c.move_joints([float(x) for x in a[1].split(",")], float(a[2]) if len(a) > 2 else 3.0)
    elif cmd == "grip":
        c.grip(a[1])
    elif cmd == "servo":
        c.servo(list(map(float, a[1:4])), int(a[4]) if len(a) > 4 else 20)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
