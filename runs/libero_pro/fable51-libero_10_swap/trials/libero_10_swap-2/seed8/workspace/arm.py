#!/usr/bin/env python3
"""Arm helper: IK in world frame (TCP), trajectory, gripper, state readers.

Usage as CLI:
  python3 arm.py js                                  # joint state (arm + fingers)
  python3 arm.py hand                                # world->hand and TCP pose
  python3 arm.py ik   x y z qx qy qz qw              # print IK joints (TCP pose, world frame)
  python3 arm.py move x y z qx qy qz qw [sec]        # IK + trajectory
  python3 arm.py joints j1,...,j7 [sec]              # raw trajectory
  python3 arm.py j7 <delta_rad> [sec]                # rotate joint7 by delta
  python3 arm.py grip open|close
"""
import sys
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # IK model frame == world (verified via /compute_fk)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    """Hamilton product a*b, both (x,y,z,w)."""
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joint_state(self):
        self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        js = self.joint_state()
        return [js[j] for j in JOINTS]

    def fingers(self):
        js = self.joint_state()
        return js["panda_finger_joint1"], js["panda_finger_joint2"]

    def hand_pose(self):
        """world->panda_hand (pos, quat) and TCP world position."""
        for _ in range(50):
            self.spin(0.1)
            if self.tfbuf.can_transform("world", "panda_hand", Time()):
                break
        t = self.tfbuf.lookup_transform("world", "panda_hand", Time())
        tr, q = t.transform.translation, t.transform.rotation
        p = np.array([tr.x, tr.y, tr.z]); qq = np.array([q.x, q.y, q.z, q.w])
        R = quat_R(*qq)
        return p, qq, p + TCP * R[:, 2]

    def solve_ik(self, tcp_world, quat, seed=None, timeout=60):
        """IK for the hand such that the TCP sits at tcp_world (world frame).
        quat is the desired panda_hand orientation; the IK tip is panda_link8
        (= hand rotated +45 deg about z), so convert."""
        R = quat_R(*quat)
        hand_world = np.asarray(tcp_world, float) - TCP * R[:, 2]
        hand_base = hand_world - BASE_IN_WORLD
        quat = quat_mul(quat, [0.0, 0.0, 0.3826834, 0.9238795])
        if not self.ik.wait_for_service(timeout_sec=10):
            raise RuntimeError("IK service unavailable")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed_js = JointState()
        seed_js.name = list(JOINTS)
        seed_js.position = [float(v) for v in (seed if seed is not None else self.arm_q())]
        req.ik_request.robot_state.joint_state = seed_js
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            raise RuntimeError(f"IK failed code={res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def traj(self, points, seconds):
        """points: list of 7-vectors; seconds: list of time_from_start."""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for q, s in zip(points, seconds):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(s), nanosec=int((s % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q = self.arm_q()
        err = np.abs(np.array(q) - np.array(points[-1])).max()
        print(f"traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_to(self, q, seconds=3.0):
        return self.traj([q], [seconds])

    def move_tcp(self, tcp_world, quat, seconds=3.0, seed=None):
        q = self.solve_ik(tcp_world, quat, seed)
        r = self.move_to(q, seconds)
        p, qq, tcp = self.hand_pose()
        print(f"TCP now {np.round(tcp, 4)} q={np.round(qq, 4)}")
        return r

    def grip(self, width):
        if not self.gr.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"grip -> reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f} gap={abs(f[0]) + abs(f[1]):.4f}")
        return r


def main():
    a = sys.argv[1:]
    arm = Arm()
    if a[0] == "js":
        print(arm.joint_state())
    elif a[0] == "hand":
        p, q, tcp = arm.hand_pose()
        print("hand", np.round(p, 4), "q", np.round(q, 4), "tcp", np.round(tcp, 4))
    elif a[0] == "ik":
        print(",".join(f"{v:.5f}" for v in arm.solve_ik([float(v) for v in a[1:4]], [float(v) for v in a[4:8]])))
    elif a[0] == "move":
        sec = float(a[8]) if len(a) > 8 else 3.0
        arm.move_tcp([float(v) for v in a[1:4]], [float(v) for v in a[4:8]], sec)
    elif a[0] == "joints":
        sec = float(a[2]) if len(a) > 2 else 3.0
        arm.move_to([float(v) for v in a[1].split(",")], sec)
    elif a[0] == "j7":
        sec = float(a[2]) if len(a) > 2 else 3.0
        q = arm.arm_q(); q[6] += float(a[1])
        arm.move_to(q, sec)
        print("joints now", np.round(arm.arm_q(), 4))
    elif a[0] == "grip":
        arm.grip(GRIP["open_m"] if a[1] == "open" else GRIP["closed_m"])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
