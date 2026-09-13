#!/usr/bin/env python3
"""Control helpers with one persistent node: IK, trajectory, gripper, servo, state."""
import math
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034
DOWN = (1.0, 0.0, 0.0, 0.0)  # hand z down, fingers along world y


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def down_yaw(yaw):
    """Quaternion: hand z down, rotated by yaw about world z (yaw=0 -> fingers along world y)."""
    # q = Rz(yaw) * Rx(pi)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sy,cy); product (w1w2 - v1.v2, ...)
    # Rz*Rx: w = cy*0 - 0 = 0 ; v = cy*(1,0,0) + 0 + (0,0,sy)x(1,0,0) = (cy, sy, 0)
    return (cy, sy, 0.0, 0.0)


class Ctl:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("ctl")
        self.js = {}
        self.n.create_subscription(JointState, "/joint_states", self._js, 10)
        self.ik = self.n.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.n.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.n, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.n, GripperCommand, "/franka_gripper/gripper_action")
        self.tw = self.n.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        while "m" not in self.js:
            rclpy.spin_once(self.n, timeout_sec=0.2)

    def _js(self, m):
        self.js["m"] = m

    def joints(self, fresh=True):
        if fresh:
            self.js.pop("m", None)
            while "m" not in self.js:
                rclpy.spin_once(self.n, timeout_sec=0.2)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[a] for a in ARM]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def hand_pose(self):
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        j = self.joints()
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = [j[a] for a in ARM]
        f = self.fk.call_async(req); rclpy.spin_until_future_complete(self.n, f, timeout_sec=30)
        p = f.result().pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z]), (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp_pose(self):
        p, q = self.hand_pose()
        R = quat_R(*q)
        return p + TCP * R[:, 2], q

    def solve_ik(self, xyz, q, at_tcp=True, seed=None):
        xyz = np.array(xyz, float)
        if at_tcp:
            xyz = xyz - TCP * quat_R(*q)[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = xyz
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = seed if seed is not None else self.arm_q()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        f = self.ik.call_async(req); rclpy.spin_until_future_complete(self.n, f, timeout_sec=60)
        r = f.result()
        if r is None or r.error_code.val != 1:
            print("IK failed", None if r is None else r.error_code.val); return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[a] for a in ARM]

    def traj(self, points, secs, tol=0.01, retries=4):
        """points: list of 7-vectors; secs: list of time_from_start.
        Resends the final target until joints converge within tol."""
        for attempt in range(retries):
            g = FollowJointTrajectory.Goal()
            g.trajectory.joint_names = ARM
            for p, t in zip(points, secs):
                pt = JointTrajectoryPoint(positions=[float(v) for v in p])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                g.trajectory.points.append(pt)
            f = self.fjt.send_goal_async(g); rclpy.spin_until_future_complete(self.n, f)
            rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.n, rf)
            code = rf.result().result.error_code
            q = self.arm_q()
            err = max(abs(a - b) for a, b in zip(q, points[-1]))
            print(f"traj[{attempt}] error_code={code} max joint err={err:.4f}")
            if err < tol:
                break
            points, secs = [points[-1]], [max(1.5, secs[-1] / 2)]
        return code, err

    def move_tcp(self, xyz, q=DOWN, secs=3.0, seed=None):
        sol = self.solve_ik(xyz, q, seed=seed)
        if sol is None:
            return False
        self.traj([sol], [secs])
        p, _ = self.tcp_pose()
        print("tcp now", p.round(4), "target", np.array(xyz).round(4))
        return True

    def move_tcp_lin(self, xyz, q=DOWN, step=0.02, speed=0.05, max_jump=0.25):
        """Straight-line TCP move: IK at waypoints every `step` m, each seeded
        with the previous solution and required to stay on the same branch."""
        start, _ = self.tcp_pose()
        goal = np.array(xyz, float)
        d = np.linalg.norm(goal - start)
        nseg = max(1, int(math.ceil(d / step)))
        seed = self.arm_q()
        pts, secs = [], []
        dt = max(0.5, (d / nseg) / speed)
        for i in range(1, nseg + 1):
            wp = start + (goal - start) * i / nseg
            sol = None
            for _ in range(6):
                s = self.solve_ik(wp, q, seed=seed)
                if s is not None and max(abs(a - b) for a, b in zip(s, seed)) < max_jump:
                    sol = s; break
            if sol is None:
                print(f"lin: no continuous IK at waypoint {i}/{nseg} {wp.round(4)}; stopping before it")
                break
            pts.append(sol); secs.append(dt * len(pts)); seed = sol
        if not pts:
            return False
        self.traj(pts, secs)
        p, _ = self.tcp_pose()
        print("tcp now", p.round(4), "target", goal.round(4))
        return len(pts) == nseg

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = 30.0
        f = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.n, f, timeout_sec=30)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.n, rf, timeout_sec=120)
        r = rf.result().result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}")

    def servo(self, v, ticks, frame="panda_link0"):
        msg = TwistStamped(); msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = v
        for _ in range(ticks):
            msg.header.stamp = self.n.get_clock().now().to_msg()
            self.tw.publish(msg); rclpy.spin_once(self.n, timeout_sec=0.05)
        p, _ = self.tcp_pose(); print("tcp after servo", p.round(4))


if __name__ == "__main__":
    c = Ctl()
    print("joints", {k: round(v, 4) for k, v in c.joints().items()})
    p, q = c.hand_pose(); print("hand", p.round(4), np.round(q, 4))
    p, q = c.tcp_pose(); print("tcp", p.round(4))
    sol = c.solve_ik(p, q, at_tcp=True)
    print("ik roundtrip", None if sol is None else np.round(sol, 4))
