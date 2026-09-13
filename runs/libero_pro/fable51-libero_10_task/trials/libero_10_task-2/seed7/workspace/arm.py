#!/usr/bin/env python3
"""Small arm controller for this machine (Franka Panda, MoveIt IK).

Usage:
  python3 arm.py tcp X Y Z QX QY QZ QW [SECONDS]   # move fingertip point (world frame)
  python3 arm.py joints J1,...,J7 [SECONDS]        # joint-space move
  python3 arm.py j7 DELTA [SECONDS]                # rotate joint7 by DELTA rad, others held
  python3 arm.py grip WIDTH                        # per-finger position (0.04 open, 0.0 closed)
  python3 arm.py state                             # joints + hand/tcp pose (world)
World -> base conversion uses TF world->panda_link0; IK requests leave
frame_id empty (planner model frame == arm base).
"""
import sys
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import JointTolerance
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self):
        rclpy.init()
        self.n = rclpy.create_node("arm_ctl")
        self.js = {}
        self.n.create_subscription(JointState, "/joint_states",
                                   lambda m: self.js.__setitem__("m", m), 1)
        self.buf = Buffer()
        TransformListener(self.buf, self.n)
        self.fjt = ActionClient(self.n, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.n, GripperCommand, GRIP["port"])
        self.ik = self.n.create_client(GetPositionIK, M["planning"]["ik_service"])

    def spin(self, t=0.1):
        rclpy.spin_once(self.n, timeout_sec=t)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            self.spin()
        m = self.js["m"]
        return dict(zip(m.name, m.position))

    def tf(self, parent, child):
        while not self.buf.can_transform(parent, child, rclpy.time.Time()):
            self.spin()
        t = self.buf.lookup_transform(parent, child, rclpy.time.Time()).transform
        p = np.array([t.translation.x, t.translation.y, t.translation.z])
        q = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        return p, q

    def state(self):
        j = self.joints()
        print("joints:", ",".join(f"{j[n]:.4f}" for n in JOINTS))
        print("fingers:", j.get("panda_finger_joint1"), j.get("panda_finger_joint2"))
        p, q = self.tf("world", "panda_hand")
        tcp = p + TCP * quat_R(*q)[:, 2]
        print(f"hand(world): {p.round(4)} q {np.round(q, 4)}")
        print(f"tcp(world):  {tcp.round(4)}")

    def solve_ik(self, hand_world, q):
        # verified via /compute_fk: the planner's model frame is `world`
        # on this machine (FK of panda_hand == TF world->panda_hand)
        p = np.asarray(hand_world)
        if not self.ik.wait_for_service(timeout_sec=10):
            raise SystemExit("no IK service")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = "panda_hand"
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, q)
        req.ik_request.avoid_collisions = False
        cur = self.joints()
        seed = JointState()
        for n in JOINTS:
            seed.name.append(n)
            seed.position.append(cur[n])
        req.ik_request.robot_state.joint_state = seed
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"IK FAILED code={None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[n] for n in JOINTS]

    def move_joints(self, positions, seconds):
        lim = FJT["limits_rad"]
        for p, (lo, hi) in zip(positions, lim):
            if not lo <= p <= hi:
                raise SystemExit(f"target {p} outside limit [{lo},{hi}]")
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise SystemExit("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        # loads (e.g. turning a knob) make the controller lag; do not let
        # the path tolerance abort the goal, only judge the final state
        for n in JOINTS:
            goal.path_tolerance.append(JointTolerance(name=n, position=3.0))
            goal.goal_tolerance.append(JointTolerance(name=n, position=0.05))
        goal.goal_time_tolerance = Duration(sec=10)
        f = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, f)
        r = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, r)
        code = r.result().result.error_code
        cur = self.joints()
        err = max(abs(cur[n] - p) for n, p in zip(JOINTS, positions))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code

    def move_tcp(self, xyz, q, seconds):
        R = quat_R(*q)
        hand = np.asarray(xyz, float) - TCP * R[:, 2]
        sol = self.solve_ik(hand, q)
        print("ik:", ",".join(f"{x:.4f}" for x in sol))
        self.move_joints(sol, seconds)
        self.state()

    def gripper(self, width):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise SystemExit("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=30)
        r = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, r, timeout_sec=120)
        res = r.result().result
        j = self.joints()
        print(f"gripper reached={res.reached_goal} stalled={res.stalled} "
              f"fingers={j['panda_finger_joint1']:.4f},{j['panda_finger_joint2']:.4f}")


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    arm = Arm()
    cmd = a[0]
    if cmd == "state":
        arm.state()
    elif cmd == "tcp":
        xyz = list(map(float, a[1:4])); q = list(map(float, a[4:8]))
        sec = float(a[8]) if len(a) > 8 else 3.0
        arm.move_tcp(xyz, q, sec)
    elif cmd == "joints":
        pos = list(map(float, a[1].split(",")))
        sec = float(a[2]) if len(a) > 2 else 3.0
        arm.move_joints(pos, sec); arm.state()
    elif cmd == "j7":
        d = float(a[1]); sec = float(a[2]) if len(a) > 2 else 2.0
        cur = arm.joints(); pos = [cur[n] for n in JOINTS]; pos[6] += d
        arm.move_joints(pos, sec); arm.state()
    elif cmd == "grip":
        arm.gripper(float(a[1]))
    else:
        raise SystemExit(__doc__)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
