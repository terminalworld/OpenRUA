#!/usr/bin/env python3
"""Small pick/place driver for this Panda. Poses are given in the WORLD
frame for the TCP (fingertip point), converted to the arm base for IK.

Usage: python3 arm.py <cmd> [args] ...   (several commands in sequence)
  goto X Y Z [T]      TCP to world (X,Y,Z), hand pointing down, fingers
                      along world y; T seconds (default 3)
  gotoyaw X Y Z YAW [T]  same with the hand yawed YAW rad about world z
  open | close        gripper
  js                  print arm joints + finger gap
"""
import math, sys, time
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GR = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE = np.array([0.0, 0.0, 0.0])  # FK/IK model frame already == world (checked with /compute_fk)


class Arm:
    def __init__(self):
        self.node = rclpy.create_node("arm")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GR["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self.js["m"]
        d = dict(zip(m.name, m.position))
        return [d[j] for j in JOINTS], d

    def gap(self):
        _, d = self.joints()
        return abs(d["panda_finger_joint1"]) + abs(d["panda_finger_joint2"])

    def solve(self, xyz_world, yaw=0.0):
        # hand pointing down: 180deg about x, then yaw about world z
        # IK tip link is panda_link8, which sits 45deg (about z) off
        # panda_hand; compensate so `yaw` is the HAND's world yaw
        yaw8 = yaw - math.pi / 4
        cy, sy = math.cos(yaw8 / 2), math.sin(yaw8 / 2)
        # q = qz(yaw) * qx(pi)  -> (x,y,z,w)
        q = (cy, sy, 0.0, 0.0)  # derived: qz*qx with qx=(1,0,0,0)
        R = _R(*q)
        hand = np.array(xyz_world) - TCP * R[:, 2]
        p = hand - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = q
        cur, _ = self.joints()
        seed = JointState(); seed.name = list(JOINTS); seed.position = cur
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"IK failed for {xyz_world}: "
                             f"{None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move(self, positions, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        cur, _ = self.joints()
        err = max(abs(a - b) for a, b in zip(cur, positions))
        print(f"  move: error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def goto(self, xyz, yaw=0.0, secs=3.0):
        print(f"goto {xyz} yaw={yaw:.2f}", flush=True)
        sol = self.solve(xyz, yaw)
        code, err = self.move(sol, secs)
        if err > 0.02:
            print("  retrying same goal", flush=True)
            self.move(sol, secs)

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GR["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        print(f"gripper {width}: reached={r.reached_goal} stalled={r.stalled} "
              f"gap={self.gap():.4f}", flush=True)


def _R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    rclpy.init()
    arm = Arm()
    a = sys.argv[1:]
    while a:
        c = a.pop(0)
        if c == "goto":
            x, y, z = map(float, a[:3]); a = a[3:]
            t = float(a.pop(0)) if a and _isnum(a[0]) else 3.0
            arm.goto((x, y, z), 0.0, t)
        elif c == "gotoyaw":
            x, y, z, yaw = map(float, a[:4]); a = a[4:]
            t = float(a.pop(0)) if a and _isnum(a[0]) else 3.0
            arm.goto((x, y, z), yaw, t)
        elif c == "open":
            arm.gripper(GR["open_m"])
        elif c == "close":
            arm.gripper(GR["closed_m"])
        elif c == "js":
            j, d = arm.joints()
            print("joints", [round(v, 3) for v in j], "gap", round(arm.gap(), 4))
        else:
            raise SystemExit(f"unknown cmd {c}")
    rclpy.shutdown()


def _isnum(s):
    try: float(s); return True
    except ValueError: return False


if __name__ == "__main__":
    main()
