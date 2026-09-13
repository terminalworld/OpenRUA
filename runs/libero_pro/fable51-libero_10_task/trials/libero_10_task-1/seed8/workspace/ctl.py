#!/usr/bin/env python3
"""Small verified-motion controller for this Panda.

Usage:
  python3 ctl.py state                       # joints, hand & TCP pose (world)
  python3 ctl.py goto X Y Z [yaw_deg] [secs] # TCP (fingertip point) to world pose, top-down
  python3 ctl.py line X Y Z [yaw_deg] [secs] # same, but as a multi-waypoint straight-ish path
  python3 ctl.py grip open|close
World -> panda_link0 offset comes from TF; IK is asked in the planner's
model frame (link0) with an empty frame_id, seeded with the arm joints.
"""
import math
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
W2B = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 (verified via TF)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def topdown_quat(yaw_deg):
    """Hand z down; yaw rotates fingers (they close along hand y)."""
    h = math.radians(yaw_deg) / 2
    # qz(yaw) * (1,0,0,0)  -> (cos h, sin h, 0, 0) in (x,y,z,w)? compute properly
    # qz = (0,0,sin h,cos h); q0 = (1,0,0,0)
    # product qz*q0: w = -0, x = cos h, y = sin h, z = 0
    return (math.cos(h), math.sin(h), 0.0, 0.0)


class Ctl:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("ctl")
        self.js = {}
        self.n.create_subscription(JointState, "/joint_states",
                                   lambda m: self.js.update(zip(m.name, m.position)), 10)
        self.buf = Buffer()
        TransformListener(self.buf, self.n)
        self.ik = self.n.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.n, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.n, GripperCommand, GRIP["port"])
        t0 = time.time()
        while (not self.js or time.time() - t0 < 1.0) and time.time() - t0 < 10:
            rclpy.spin_once(self.n, timeout_sec=0.1)

    def spin(self, s=0.5):
        t0 = time.time()
        while time.time() - t0 < s:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def arm_q(self):
        return [self.js[j] for j in ARM]

    def hand_pose(self):
        self.spin(0.3)
        t0 = time.time()
        while not self.buf.can_transform("world", "panda_hand", rclpy.time.Time()) \
                and time.time() - t0 < 20:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        t = self.buf.lookup_transform("world", "panda_hand", rclpy.time.Time()).transform
        p = np.array([t.translation.x, t.translation.y, t.translation.z])
        R = quat_to_R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        return p, R, (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)

    def state(self):
        self.spin(0.5)
        q = self.arm_q()
        p, R, quat = self.hand_pose()
        tcp = p + TCP * R[:, 2]
        print("joints:", np.round(q, 4).tolist())
        print("fingers:", round(self.js.get("panda_finger_joint1", -1), 4),
              round(self.js.get("panda_finger_joint2", -1), 4))
        print("hand world: %s quat %s" % (np.round(p, 4), np.round(quat, 4)))
        print("tcp  world: %s" % np.round(tcp, 4))

    def solve_ik(self, tcp_world, yaw_deg, seed=None):
        if not self.ik.wait_for_service(timeout_sec=10):
            raise SystemExit("IK service unavailable")
        qx, qy, qz, qw = topdown_quat(yaw_deg)
        R = quat_to_R(qx, qy, qz, qw)
        hand_world = np.array(tcp_world) - TCP * R[:, 2]
        # verified on this machine: world-frame coords resolve correctly with
        # frame_id "world"; naming the tip link gives the HAND (not link8) pose
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = "world"
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, hand_world)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = qx, qy, qz, qw
        s = JointState()
        s.name = list(ARM)
        s.position = [float(v) for v in (seed if seed is not None else self.arm_q())]
        req.ik_request.robot_state.joint_state = s
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            print("IK no answer within 60s")
            return None
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def solve_ik_near(self, tcp_world, yaw_deg, seed, tries=6, tol=0.5):
        """IK, but insist on the solution branch nearest the seed."""
        best = None
        for _ in range(tries):
            q = self.solve_ik(tcp_world, yaw_deg, seed)
            if q is None:
                continue
            d = float(np.abs(np.array(q) - np.array(seed)).max())
            if best is None or d < best[0]:
                best = (d, q)
            if d < tol:
                break
        if best is None:
            return None
        if best[0] >= tol:
            print(f"warning: nearest IK branch is {best[0]:.2f} rad from seed")
        return best[1]

    def send_traj(self, points, secs):
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        n = len(points)
        for i, q in enumerate(points):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            t = secs * (i + 1) / n
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, send)
        gh = send.result()
        if not gh.accepted:
            raise SystemExit("FJT goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.n, rf, timeout_sec=600)
        code = rf.result().result.error_code
        self.spin(0.5)
        err = np.abs(np.array(self.arm_q()) - np.array(points[-1])).max()
        print(f"traj done error_code={code} max_joint_err={err:.4f}")
        return code, err

    def goto(self, x, y, z, yaw=0.0, secs=3.0, waypoints=1):
        target = np.array([x, y, z])
        p0, R0, _ = self.hand_pose()
        tcp0 = p0 + TCP * R0[:, 2]
        seed = self.arm_q()
        pts = []
        for i in range(1, waypoints + 1):
            a = i / waypoints
            wp = tcp0 + a * (target - tcp0)
            q = self.solve_ik_near(wp, yaw, seed)
            if q is None:
                print(f"IK FAILED at waypoint {i}/{waypoints} {np.round(wp,3)}; no motion")
                return False
            pts.append(q)
            seed = q
        code, err = self.send_traj(pts, secs)
        p, R, _ = self.hand_pose()
        tcp = p + TCP * R[:, 2]
        print("tcp now %s (target %s) dist %.4f" % (np.round(tcp, 4), np.round(target, 4),
                                                   np.linalg.norm(tcp - target)))
        return code == 0

    def grip(self, what):
        if not self.gr.wait_for_server(timeout_sec=10):
            raise SystemExit("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(GRIP["open_m"] if what == "open" else GRIP["closed_m"])
        g.command.max_effort = float(GRIP["max_effort"])
        f = self.gr.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, rf, timeout_sec=300)
        r = rf.result().result
        self.spin(1.0)
        print(f"grip {what}: reached={r.reached_goal} stalled={r.stalled} pos={r.position:.4f} "
              f"fingers={self.js.get('panda_finger_joint1'):.4f},{self.js.get('panda_finger_joint2'):.4f}")


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    c = Ctl()
    if a[0] == "state":
        c.state()
    elif a[0] in ("goto", "line"):
        x, y, z = map(float, a[1:4])
        yaw = float(a[4]) if len(a) > 4 else 0.0
        secs = float(a[5]) if len(a) > 5 else 3.0
        c.goto(x, y, z, yaw, secs, waypoints=1 if a[0] == "goto" else 4)
        c.state()
    elif a[0] == "grip":
        c.grip(a[1])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
