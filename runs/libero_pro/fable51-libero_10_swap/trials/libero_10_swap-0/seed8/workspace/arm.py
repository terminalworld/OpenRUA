#!/usr/bin/env python3
"""Arm helper for this Panda: FK, IK-goto, gripper, joint read.

Usage:
  python3 arm.py fk                          # print hand + tcp pose (base frame)
  python3 arm.py js                          # print joint state
  python3 arm.py goto x y z [qx qy qz qw] [--tcp] [--sec S]   # base frame
  python3 arm.py grip open|close
  python3 arm.py lin x y z [--tcp] [--sec S] [--steps N]   # straight-line, keeps orientation
All poses in the planning frame (panda_link0). World = base + (-0.51, 0, 0.42).
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
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
# default "hand pointing down" orientation, set from FK of the home pose
DOWN_Q = None


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.update(zip(m.name, m.position)), 1)
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.wait_js()

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        end = time.time() + 20
        while time.time() < end and not all(j in self.js for j in JOINTS):
            self.spin(0.2)
        if not all(j in self.js for j in JOINTS):
            raise SystemExit("no /joint_states")

    def arm_q(self):
        self.js.clear()
        self.wait_js()
        return [self.js[j] for j in JOINTS]

    def fingers(self):
        self.js.clear()
        self.wait_js()
        return self.js.get("panda_finger_joint1"), self.js.get("panda_finger_joint2")

    def seed_state(self, q=None):
        s = JointState()
        s.name = list(JOINTS)
        s.position = list(q if q is not None else self.arm_q())
        return s

    def fk(self, q=None):
        self.fk_cli.wait_for_service(10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.seed_state(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q

    def ik(self, pos, q, seed=None, at_tcp=False):
        pos = np.array(pos, float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(q)[:, 2]
        # the IK tip link is panda_link8, yawed +45deg (about its z) from
        # panda_hand; convert the requested HAND orientation to link8
        from scipy.spatial.transform import Rotation as _R
        q = (_R.from_quat(q) * _R.from_euler('z', 45, degrees=True)).as_quat()
        self.ik_cli.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state = self.seed_state(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise SystemExit("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def traj(self, points, seconds):
        """points: list of joint vectors; seconds: total duration."""
        if not self.fjt.wait_for_server(10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        n = len(points)
        for i, p in enumerate(points):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(x) for x in p])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        for attempt in range(3):
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            q = np.array(self.arm_q())
            err = np.abs(q - np.array(points[-1])).max()
            print(f"traj done error_code={code} max_joint_err={err:.4f}")
            if err < 0.02:
                break
            # controller lag on the first pass (machine fact): resend the
            # final point only, from where we are
            goal.trajectory.points = [goal.trajectory.points[-1]]
            t = max(2.0, seconds / 2)
            goal.trajectory.points[0].time_from_start = Duration(
                sec=int(t), nanosec=int((t % 1) * 1e9))
            print("  resending final point")
        return code, err

    def goto(self, pos, q, at_tcp=False, seconds=4.0, steps=1):
        """IK to pose; if steps>1 interpolate straight line from current pose."""
        cur_pos, cur_q = self.fk()
        if at_tcp:
            cur_pos = cur_pos + TCP * quat_to_R(cur_q)[:, 2]
        seed = self.arm_q()
        pts = []
        for i in range(1, steps + 1):
            a = i / steps
            p = (1 - a) * cur_pos + a * np.array(pos, float)
            sol = self.ik(p, q, seed=seed, at_tcp=at_tcp)
            if sol is None:
                raise SystemExit(f"IK FAILED at step {i}/{steps} pos={np.round(p,4)}")
            pts.append(sol)
            seed = sol
        code, err = self.traj(pts, seconds)
        fpos, fq = self.fk()
        tcp = fpos + TCP * quat_to_R(fq)[:, 2]
        print(f"hand={np.round(fpos,4)} tcp={np.round(tcp,4)} q={np.round(fq,4)}")
        return code, err

    def gripper(self, width):
        if not self.grip.wait_for_server(10):
            raise SystemExit("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        for _ in range(5):
            self.spin(0.1)
        f = self.fingers()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f


def main():
    a = Arm()
    args = [x for x in sys.argv[1:] if not x.startswith("--")]
    sec = float(sys.argv[sys.argv.index("--sec") + 1]) if "--sec" in sys.argv else 4.0
    steps = int(sys.argv[sys.argv.index("--steps") + 1]) if "--steps" in sys.argv else 1
    at_tcp = "--tcp" in sys.argv
    cmd = args[0]
    if cmd == "fk":
        pos, q = a.fk()
        tcp = pos + TCP * quat_to_R(q)[:, 2]
        print("hand", np.round(pos, 4), "tcp", np.round(tcp, 4), "q(xyzw)", np.round(q, 4))
        print("R\n", np.round(quat_to_R(q), 3))
    elif cmd == "js":
        print("arm", np.round(a.arm_q(), 4), "fingers", a.fingers())
    elif cmd == "goto":
        pos = [float(x) for x in args[1:4]]
        if len(args) >= 8:
            q = [float(x) for x in args[4:8]]
        else:
            _, q = a.fk()
        a.goto(pos, q, at_tcp=at_tcp, seconds=sec, steps=steps)
    elif cmd == "lin":
        pos = [float(x) for x in args[1:4]]
        _, q = a.fk()
        a.goto(pos, q, at_tcp=at_tcp, seconds=sec, steps=max(steps, 4))
    elif cmd == "grip":
        a.gripper(GRIP["open_m"] if args[1] == "open" else GRIP["closed_m"])
    elif cmd == "joints":
        q = [float(x) for x in args[1].split(",")]
        a.traj([q], sec)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
