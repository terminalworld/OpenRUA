#!/usr/bin/env python3
"""Small helper library: one node, FK/IK/trajectory/gripper/joint reads.
World<->base offset comes from TF world->panda_link0 (constant here).

CLI:
  robot.py fk                         current hand + tcp pose in world
  robot.py goto X Y Z QX QY QZ QW [sec] [--hand]   TCP (default) target in WORLD
  robot.py grip open|close
  robot.py js
"""
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
TCP = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.660, 0.0, 0.912])  # tf world->panda_link0


def quat_mul(a, b):
    """Hamilton product, quats as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + bx * 0 + ax * bw + ay * bz - az * by,
            aw * by + ay * bw + az * bx - ax * bz,
            aw * bz + az * bw + ax * by - ay * bx,
            aw * bw - ax * bx - ay * by - az * bz)


# the IK service solves for panda_link8; panda_hand is link8 rotated -45deg
# about z, so a desired HAND orientation must be converted to link8's.
Q_HAND_TO_LINK8 = (0.0, 0.0, 0.3826834323650898, 0.9238795325112867)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_state(self):
        j = self.joints()
        js = JointState()
        js.name = list(ARM)
        js.position = [j[n] for n in ARM]
        return js

    def fk_pose(self):
        """hand pose in world: (pos, quat xyzw)"""
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.arm_state()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])  # FK answers in world
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, q

    def solve_ik(self, pos_world, q, at_tcp=True, seed=None):
        pos = np.array(pos_world, float)
        q = tuple(map(float, q))
        q8 = quat_mul(q, Q_HAND_TO_LINK8)  # ik target link is panda_link8
        if at_tcp:
            R = quat_R(*q)
            pos = pos - TCP * R[:, 2]
        # verified: this machine's IK (empty frame_id) takes WORLD coords
        self.ik.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = pos
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q8
        req.ik_request.robot_state.joint_state = seed or self.arm_state()
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            code = None if res is None else res.error_code.val
            raise RuntimeError(f"IK failed code={code} for {pos_world}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, positions, seconds=3.0):
        self.traj.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(p) for p in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        j = self.joints()
        err = max(abs(j[n] - p) for n, p in zip(ARM, positions))
        print(f"traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def goto(self, pos_world, q, seconds=3.0, at_tcp=True):
        sol = self.solve_ik(pos_world, q, at_tcp)
        return self.move_joints(sol, seconds)

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        j = self.joints()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} "
              f"fingers={j['panda_finger_joint1']:.4f},{j['panda_finger_joint2']:.4f}")
        return j["panda_finger_joint1"]


def goto_converge(r, pos, q, seconds=3.0, at_tcp=True, tol=0.01, tries=4):
    sol = r.solve_ik(pos, q, at_tcp)
    for i in range(tries):
        code, err = r.move_joints(sol, seconds)
        if err < tol:
            break
    p, qq = r.fk_pose()
    tcp = p + TCP * quat_R(*qq)[:, 2]
    print("tcp world", tcp.round(4), "err", round(err, 4))
    return tcp, err


if __name__ == "__main__":
    r = Robot()
    cmd = sys.argv[1]
    if cmd == "fk":
        pos, q = r.fk_pose()
        R = quat_R(*q)
        print("hand world", pos.round(4), "quat", np.round(q, 4))
        print("tcp world", (pos + TCP * R[:, 2]).round(4))
        print("hand axes (cols x,y,z in world):\n", R.round(3))
    elif cmd == "goto":
        a = [x for x in sys.argv[2:] if not x.startswith("--")]
        v = list(map(float, a))
        sec = v[7] if len(v) > 7 else 3.0
        r.goto(v[:3], v[3:7], sec, at_tcp="--hand" not in sys.argv)
        pos, q = r.fk_pose()
        R = quat_R(*q)
        print("now tcp world", (pos + TCP * R[:, 2]).round(4))
    elif cmd == "grip":
        r.gripper(GRIP["open_m"] if sys.argv[2] == "open" else GRIP["closed_m"])
    elif cmd == "js":
        print(r.joints())
    rclpy.shutdown()


def wrench(r):
    from geometry_msgs.msg import WrenchStamped
    port = next(s for s in M["sensors"] if s["kind"] == "wrench")["port"]
    got = {}
    sub = r.node.create_subscription(WrenchStamped, port, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        r.spin(0.2)
    r.node.destroy_subscription(sub)
    w = got["m"].wrench
    return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]).round(3)


def tcp_now(r):
    p, q = r.fk_pose()
    return p + TCP * quat_R(*q)[:, 2]


def goto_cl(r, target, q, seconds=2.0, tol=0.003, iters=5, at_tcp=True):
    """Closed-loop: command, read FK, offset the command by the residual.
    Compensates the controller's steady-state (gravity) sag."""
    target = np.array(target, float)
    cmd = target.copy()
    for i in range(iters):
        sol = r.solve_ik(cmd, q, at_tcp)
        for _ in range(3):
            code, err = r.move_joints(sol, seconds)
            if err < 0.01:
                break
        actual = tcp_now(r)
        e = target - actual
        print(f"  iter{i} actual {actual.round(4)} resid {np.linalg.norm(e)*1000:.1f}mm")
        if np.linalg.norm(e) < tol:
            break
        cmd = cmd + e
    return actual
