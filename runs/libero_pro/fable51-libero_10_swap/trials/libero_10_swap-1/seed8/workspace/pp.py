#!/usr/bin/env python3
"""Pick-and-place helper: one node, reused clients, verify after each step.

Usage: python3 -u pp.py <cmd> [args]
  open | close                      gripper
  goto <wx> <wy> <wz> [secs]        move TCP to WORLD xyz, hand pointing down
  pose                              print hand/TCP world pose + finger gap
  pick <wx> <wy> <wz_grasp>         open, hover, descend, close, lift, report
  place <wx> <wy> <wz>              hover over target, open, report
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
TCP_OFF = float(M["hand"]["tcp_offset_m"])
# world -> panda_link0 (read from TF at startup; fallback measured)
BASE_IN_WORLD = np.array([-0.510, 0.0, 0.420])
# IK tip link is panda_link8 (= panda_hand yawed +45deg about z). This is
# link8's orientation for hand pointing straight down, fingers along world Y.
Q_DOWN = (0.9238795, -0.3826834, 0.0, 0.0)
HOVER_Z = 0.60


class PP:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("pp")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fjt.wait_for_server(10), "no FJT"
        assert self.grip.wait_for_server(10), "no gripper"
        self.spin_until(lambda: "m" in self.js, 10)
        global BASE_IN_WORLD
        t = self.tf("world", M["frames"]["base"])
        if t is not None:
            BASE_IN_WORLD = t[0]
        print(f"base in world: {BASE_IN_WORLD}")

    def spin_until(self, pred, timeout):
        end = time.time() + timeout
        while not pred() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return pred()

    def joints(self, fresh=True):
        if fresh:
            self.js.pop("m", None)
            self.spin_until(lambda: "m" in self.js, 10)
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def tf(self, parent, child):
        for _ in range(50):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.tfbuf.can_transform(parent, child, rclpy.time.Time()):
                break
        else:
            return None
        t = self.tfbuf.lookup_transform(parent, child, rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), (q.x, q.y, q.z, q.w)

    def pose(self):
        # TF for the arm chain is sim-stamped; force a fresh listener read
        hp = self.tf("world", M["frames"]["hand"])
        j = self.joints()
        gap = j["panda_finger_joint1"] + abs(j["panda_finger_joint2"])
        if hp is None:
            print("hand pose: TF unavailable")
            return None
        p, q = hp
        R = quat_R(*q)
        tcp = p + TCP_OFF * R[:, 2]
        print(f"hand world {np.round(p, 4)} q {np.round(q, 3)}  TCP world {np.round(tcp, 4)}"
              f"  finger gap {gap:.4f}")
        return tcp, gap

    def solve_ik(self, world_xyz, q=Q_DOWN):
        # target for the HAND frame: TCP shifted back along hand +Z
        R = quat_R(*q)
        hand_w = np.array(world_xyz) - TCP_OFF * R[:, 2]
        # verified: with empty frame_id this machine's IK takes WORLD coords
        hand_b = hand_w
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        # Seed from the SRDF "ready" pose with joint1 aimed at the target:
        # seeding from the current config can return a twisted branch
        # (huge joint travel, tolerance violations).
        d = hand_w - BASE_IN_WORLD
        j1 = float(np.arctan2(d[1], d[0]))
        natural = np.array([j1, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        best = None
        rng = np.random.default_rng(0)
        for attempt in range(8):
            seedv = natural if attempt == 0 else natural + rng.normal(0, 0.3, 7)
            seed = JointState()
            seed.name = list(ARM)
            seed.position = [float(v) for v in seedv]
            req.ik_request.robot_state.joint_state = seed
            fut = self.ik.call_async(req)
            self.spin_until(fut.done, 60)
            res = fut.result()
            if res is None or res.error_code.val != 1:
                print(f"  IK attempt {attempt} failed: "
                      f"{None if res is None else res.error_code.val}")
                continue
            sol = dict(zip(res.solution.joint_state.name,
                           res.solution.joint_state.position))
            solv = np.array([sol[j] for j in ARM])
            dev = np.max(np.abs(solv - natural))
            if best is None or dev < best[0]:
                best = (dev, solv)
            if dev < 1.0:
                break
        if best is None:
            raise SystemExit(f"IK failed for world TCP {world_xyz}")
        print(f"  IK sol {np.round(best[1], 3)} (dev from natural {best[0]:.2f})")
        return list(best[1])

    def move_joints(self, positions, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        f = self.fjt.send_goal_async(goal)
        self.spin_until(f.done, 60)
        rf = f.result().get_result_async()
        self.spin_until(rf.done, 600)
        code = rf.result().result.error_code
        cur = self.joints()
        err = max(abs(cur[j] - p) for j, p in zip(ARM, positions))
        print(f"  traj error_code={code} max joint err={err:.4f}")
        return code, err

    def goto(self, world_xyz, secs=3.0, q=Q_DOWN):
        print(f"goto TCP world {np.round(world_xyz, 4)}")
        sol = self.solve_ik(world_xyz, q)
        cur = self.joints()
        travel = max(abs(cur[j] - p) for j, p in zip(ARM, sol))
        secs = max(secs, travel / 0.3)  # cap joint speed ~0.3 rad/s
        print(f"  travel {travel:.2f} rad -> {secs:.1f}s")
        code, err = self.move_joints(sol, secs)
        for _ in range(4):
            if code == 0 and err < 0.01:
                break
            print("  resending same goal (controller lag)")
            code, err = self.move_joints(sol, max(2.0, secs / 2))
        return self.pose()

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(goal)
        self.spin_until(f.done, 30)
        rf = f.result().get_result_async()
        self.spin_until(rf.done, 300)
        r = rf.result().result
        j = self.joints()
        gap = j["panda_finger_joint1"] + abs(j["panda_finger_joint2"])
        print(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} "
              f"gap={gap:.4f}")
        return gap

    def pick(self, x, y, zg):
        self.gripper(GRIP["open_m"])
        self.goto((x, y, HOVER_Z), 4.0)
        self.goto((x, y, zg + 0.08), 2.0)
        self.goto((x, y, zg), 2.0)
        gap = self.gripper(GRIP["closed_m"])
        self.goto((x, y, HOVER_Z), 2.5)
        gap2 = self.joints()
        g = gap2["panda_finger_joint1"] + abs(gap2["panda_finger_joint2"])
        print(f"PICK RESULT: gap after close {gap:.4f}, after lift {g:.4f} "
              f"({'HOLDING' if g > 0.005 else 'EMPTY'})")
        return g > 0.005

    def place(self, x, y, z):
        self.goto((x, y, z), 4.0)
        self.gripper(GRIP["open_m"])
        self.pose()


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    pp = PP()
    cmd = a[0]
    if cmd == "open":
        pp.gripper(GRIP["open_m"])
    elif cmd == "close":
        pp.gripper(GRIP["closed_m"])
    elif cmd == "pose":
        pp.pose()
    elif cmd == "goto":
        secs = float(a[4]) if len(a) > 4 else 3.0
        pp.goto(tuple(map(float, a[1:4])), secs)
    elif cmd == "pick":
        pp.pick(*map(float, a[1:4]))
    elif cmd == "place":
        pp.place(*map(float, a[1:4]))
    else:
        raise SystemExit(__doc__)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
